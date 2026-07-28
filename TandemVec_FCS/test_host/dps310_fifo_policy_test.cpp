#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <vector>

#include "../lib/DPS310-Pressure-Sensor/src/Dps310FifoPolicy.h"

namespace
{
void require(bool condition, const char *message)
{
  if (!condition)
  {
    std::fprintf(stderr, "FAILED: %s\n", message);
    std::exit(1);
  }
}

struct Sample
{
  int16_t type;
  int32_t raw;
};

struct MockFifo
{
  std::vector<Sample> samples;
  std::size_t index = 0;
  int empty_checks = 0;
  int reads = 0;

  bool empty() const
  {
    return index >= samples.size();
  }

  int16_t read(int32_t &raw)
  {
    reads++;
    if (empty())
    {
      return -1;
    }
    raw = samples[index].raw;
    return samples[index++].type;
  }
};

void returns_latest_complete_temperature_and_pressure_pair()
{
  MockFifo fifo{{{0, 100}, {1, 1000}, {0, 200}, {1, 2000}, {0, 300}, {1, 3000}}};
  dps310::LatestFifoRawResult result{};

  const int16_t status = dps310::readLatestRawFromFifo(
      [&fifo]() {
        fifo.empty_checks++;
        return fifo.empty();
      },
      [&fifo](int32_t &raw) {
        return fifo.read(raw);
      },
      result);

  require(status == 0, "latest pair status");
  require(result.temp_available, "latest pair temperature available");
  require(result.pressure_available, "latest pair pressure available");
  require(result.raw_temp == 300, "latest pair temperature value");
  require(result.raw_pressure == 3000, "latest pair pressure value");
  require(fifo.reads == 6, "latest pair drains FIFO");
}

void returns_partial_latest_sample_when_only_pressure_arrives()
{
  MockFifo fifo{{{1, 1010}, {1, 2020}, {1, 3030}}};
  dps310::LatestFifoRawResult result{};

  const int16_t status = dps310::readLatestRawFromFifo(
      [&fifo]() { return fifo.empty(); },
      [&fifo](int32_t &raw) { return fifo.read(raw); },
      result);

  require(status == 0, "partial pressure status");
  require(!result.temp_available, "partial pressure has no temperature");
  require(result.pressure_available, "partial pressure available");
  require(result.raw_pressure == 3030, "partial pressure latest value");
}

void requires_pressure_sample_for_baro_update()
{
  dps310::LatestFifoRawResult temp_only{};
  temp_only.temp_available = true;
  temp_only.raw_temp = 1234;
  require(dps310::pressureUpdateStatus(temp_only) == -4, "temperature only is not a baro update");

  dps310::LatestFifoRawResult pressure_only{};
  pressure_only.pressure_available = true;
  pressure_only.raw_pressure = 5678;
  require(dps310::pressureUpdateStatus(pressure_only) == 0, "pressure sample is a baro update");
}

void reports_unfinished_when_fifo_empty()
{
  MockFifo fifo{};
  dps310::LatestFifoRawResult result{};

  const int16_t status = dps310::readLatestRawFromFifo(
      [&fifo]() { return fifo.empty(); },
      [&fifo](int32_t &raw) { return fifo.read(raw); },
      result);

  require(status == -4, "empty FIFO status");
  require(fifo.reads == 0, "empty FIFO no reads");
}

void caps_reads_to_fifo_depth()
{
  MockFifo fifo{};
  for (int i = 0; i < 40; ++i)
  {
    fifo.samples.push_back({static_cast<int16_t>(i & 1), i});
  }
  dps310::LatestFifoRawResult result{};

  const int16_t status = dps310::readLatestRawFromFifo(
      [&fifo]() { return fifo.empty(); },
      [&fifo](int32_t &raw) { return fifo.read(raw); },
      result);

  require(status == 0, "read cap status");
  require(fifo.reads == 32, "read cap count");
  require(result.temp_available, "read cap temperature available");
  require(result.pressure_available, "read cap pressure available");
  require(result.raw_temp == 30, "read cap temperature value");
  require(result.raw_pressure == 31, "read cap pressure value");
}

void supports_small_bounded_drain_for_realtime_tasks()
{
  MockFifo fifo{{{1, 1001}, {0, 2001}, {1, 1002}}};
  dps310::LatestFifoRawResult result{};

  const int16_t status = dps310::readLatestRawFromFifo(
      [&fifo]() { return fifo.empty(); },
      [&fifo](int32_t &raw) { return fifo.read(raw); },
      result,
      2);

  require(status == 0, "bounded drain status");
  require(fifo.reads == 2, "bounded drain read count");
  require(result.pressure_available, "bounded drain pressure available");
  require(result.temp_available, "bounded drain temperature available");
  require(result.raw_pressure == 1001, "bounded drain pressure value");
  require(result.raw_temp == 2001, "bounded drain temperature value");
}

void supports_trusted_first_sample_then_empty_check()
{
  MockFifo fifo{{{1, 1001}}};
  dps310::LatestFifoRawResult result{};
  bool first_check = true;

  const int16_t status = dps310::readLatestRawFromFifo(
      [&fifo, &first_check]() {
        fifo.empty_checks++;
        if (first_check)
        {
          first_check = false;
          return false;
        }
        return fifo.empty();
      },
      [&fifo](int32_t &raw) { return fifo.read(raw); },
      result,
      2);

  require(status == 0, "trusted first status");
  require(fifo.reads == 1, "trusted first reads only available sample");
  require(fifo.empty_checks == 2, "trusted first checks empty before second read");
  require(result.pressure_available, "trusted first pressure available");
  require(!result.temp_available, "trusted first no phantom temperature");
  require(result.raw_pressure == 1001, "trusted first pressure value");
}

void can_stop_after_first_pressure_sample()
{
  MockFifo fifo{{{1, 1001}, {0, 2001}}};
  dps310::LatestFifoRawResult result{};
  bool first_check = true;

  const int16_t status = dps310::readLatestRawFromFifo(
      [&fifo, &first_check, &result]() {
        fifo.empty_checks++;
        if (first_check)
        {
          first_check = false;
          return false;
        }
        if (result.pressure_available)
        {
          return true;
        }
        return fifo.empty();
      },
      [&fifo](int32_t &raw) { return fifo.read(raw); },
      result,
      2);

  require(status == 0, "stop after pressure status");
  require(fifo.reads == 1, "stop after pressure reads only first sample");
  require(fifo.empty_checks == 2, "stop after pressure performs one post-read check");
  require(result.pressure_available, "stop after pressure has pressure");
  require(!result.temp_available, "stop after pressure skips later temperature");
}

void pressure_priority_policy_stops_only_after_pressure()
{
  dps310::LatestFifoRawResult temp_only{};
  temp_only.temp_available = true;
  require(!dps310::shouldStopFifoReadAfterSample(temp_only, true), "temperature sample does not stop pressure-priority read");

  dps310::LatestFifoRawResult pressure_ready{};
  pressure_ready.pressure_available = true;
  require(dps310::shouldStopFifoReadAfterSample(pressure_ready, true), "pressure sample stops pressure-priority read");
  require(!dps310::shouldStopFifoReadAfterSample(pressure_ready, false), "disabled policy keeps bounded drain behavior");
}

void max_reads_one_temperature_first_is_not_a_baro_update()
{
  // 单样本路径（BFS_DPS310_FIFO_MAX_READS=1）：如果 FIFO 中第一个样本是温度，
  // readLatestRawFromFifo 在策略层返回 SUCCEEDED（读到了数据），
  // 但 pressureUpdateStatus 返回 FAIL_UNFINISHED（没有压力数据）。
  // 这是正常情况：温度 8Hz / 压力 128Hz，偶尔轮到温度样本排在最前面。
  MockFifo fifo{{{0, 999}, {1, 1001}}};
  dps310::LatestFifoRawResult result{};

  const int16_t status = dps310::readLatestRawFromFifo(
      [&fifo]() { return fifo.empty(); },
      [&fifo](int32_t &raw) { return fifo.read(raw); },
      result,
      1U);

  require(status == 0, "max1 temp-first policy-level status");
  require(fifo.reads == 1, "max1 reads exactly one sample");
  require(result.temp_available, "max1 temperature available");
  require(!result.pressure_available, "max1 no pressure from single temp sample");
  require(dps310::pressureUpdateStatus(result) == -4, "max1 temp-only is not a baro update");
}

void flushes_only_on_recoverable_fifo_errors()
{
  require(!dps310::shouldFlushFifoAfterReadStatus(0), "success should not flush");
  require(!dps310::shouldFlushFifoAfterReadStatus(-4), "unfinished should not flush");
  require(dps310::shouldFlushFifoAfterReadStatus(-1), "unknown read error should flush");
  require(dps310::shouldFlushFifoAfterReadStatus(-5), "overflow should flush");
}

void decodes_dps310_fifo_status_from_single_register_read()
{
  const dps310::FifoStatus empty = dps310::decodeDps310FifoStatus(0x01);
  require(empty.empty, "single status empty bit");
  require(!empty.full, "single status empty is not full");
  require(!empty.has_sample, "single status empty has no sample");

  const dps310::FifoStatus full = dps310::decodeDps310FifoStatus(0x02);
  require(!full.empty, "single status full is not empty");
  require(full.full, "single status full bit");
  require(full.has_sample, "single status full still has sample");

  const dps310::FifoStatus readable = dps310::decodeDps310FifoStatus(0x00);
  require(!readable.empty, "single status readable is not empty");
  require(!readable.full, "single status readable is not full");
  require(readable.has_sample, "single status readable has sample");
}

} // namespace

int main()
{
  returns_latest_complete_temperature_and_pressure_pair();
  returns_partial_latest_sample_when_only_pressure_arrives();
  requires_pressure_sample_for_baro_update();
  reports_unfinished_when_fifo_empty();
  caps_reads_to_fifo_depth();
  supports_small_bounded_drain_for_realtime_tasks();
  supports_trusted_first_sample_then_empty_check();
  can_stop_after_first_pressure_sample();
  pressure_priority_policy_stops_only_after_pressure();
  max_reads_one_temperature_first_is_not_a_baro_update();
  flushes_only_on_recoverable_fifo_errors();
  decodes_dps310_fifo_status_from_single_register_read();
  std::puts("dps310_fifo_policy_test passed");
  return 0;
}
