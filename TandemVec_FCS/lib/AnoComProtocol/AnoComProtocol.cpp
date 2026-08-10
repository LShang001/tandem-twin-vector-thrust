#include "AnoComProtocol.h"

// 全局指针，供 parseData 回调访问通信模块的发送能力。
// 在 communication.cpp 的 handleAnoCom 中通过 setRxCallback() 赋值。
// 末两参为源帧的 SC/AC 校验值（安全协议回传 0x00 校验帧用）。
static void (*g_ano_rx_callback)(uint8_t funcCode, uint8_t *data, uint16_t len,
                                 uint8_t rxSumCheck, uint8_t rxAddCheck) = nullptr;

AnoComProtocol::AnoComProtocol(Stream *serial)
{
    _serial = serial;
    _rxIndex = 0;
    _dataReceived = false;
    _groupLen = 0;
    _groupActive = false;

    // 初始化灵活格式帧数据数量
    for (int i = 0; i < 10; i++)
    {
        _flexDataCount[i] = 0;
    }
}

void AnoComProtocol::begin(long baudRate)
{
    // _serial->begin(baudRate); // 串口初始化已经在 setup() 中完成
}

/**
 * @brief 发送数据帧
 *
 * 本函数按照Ano通信协议的规范，组装数据帧并发送到串口。数据帧包括帧头、源地址、目标地址、功能码、数据长度、数据和校验和。
 *
 * @param destAddr 目标设备地址
 * @param funcCode 功能码，表示数据帧的功能或用途
 * @param data 指向待发送数据的指针
 * @param len 数据的长度，单位为字节
 */
void AnoComProtocol::sendData(uint8_t destAddr, uint8_t funcCode, uint8_t *data, uint16_t len)
{
    if (_groupActive)
    {
        // 组模式：拼入组缓冲，endGroup() 时一次 write
        if (_groupLen + len + 8 > sizeof(_groupBuf))
        {
            // 组缓冲满：先 flush 已有帧再入组（防御，正常组远小于 256B）
            _serial->write(_groupBuf, _groupLen);
            _groupLen = 0;
        }
        _groupLen += buildFrame(_groupBuf, _groupLen, destAddr, funcCode, data, len);
    }
    else
    {
        uint8_t txBuffer[ANO_MAX_DATA_LEN + 8];
        uint16_t flen = buildFrame(txBuffer, 0, destAddr, funcCode, data, len);
        _serial->write(txBuffer, flen);
    }
}

// ★ 2026-08-10 组帧模式（COMM-001 A3）：begin/end 配对使用，
//   组内多帧合并单次 write（通信侧 5 帧 → 1 次写，写竞争窗口缩小）。
void AnoComProtocol::beginGroup()
{
    _groupLen = 0;
    _groupActive = true;
}

void AnoComProtocol::endGroup()
{
    if (_groupLen > 0)
    {
        _serial->write(_groupBuf, _groupLen);
        _groupLen = 0;
    }
    _groupActive = false;
}

// ★ 2026-08-10 组帧（COMM-001 A2/A3）：与 sendData 同组装逻辑（含长度保护/校验算法），
//   但不写串口——调用方拼多帧进同一缓冲后单次 write。
//   校验单循环边拷边算（COMM-001 A2）：sum/add 同步累加，替代两遍独立遍历。
uint16_t AnoComProtocol::buildFrame(uint8_t *buf, uint16_t off, uint8_t destAddr,
                                    uint8_t funcCode, const uint8_t *data, uint16_t len)
{
    // ★ 长度保护（COMM-001 A1）：超 ANO_MAX_DATA_LEN 截断，防栈缓冲/外部缓冲越界写
    if (len > ANO_MAX_DATA_LEN)
    {
        len = ANO_MAX_DATA_LEN;
    }

    buf[off + 0] = ANO_FRAME_HEAD;
    buf[off + 1] = ANO_LOCAL_ADDR;
    buf[off + 2] = destAddr;
    buf[off + 3] = funcCode;
    buf[off + 4] = len & 0xFF;
    buf[off + 5] = (len >> 8) & 0xFF;

    // 校验单循环：头 6 字节 + 数据区边拷边算（sum=和校验, add=附加校验，与
    // calculateSumCheck/calculateAddCheck 两遍遍历结果逐字节一致）
    uint8_t sum = 0, add = 0;
    for (uint16_t i = 0; i < 6; i++)
    {
        sum += buf[off + i];
        add += sum;
    }
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t b = data[i];
        buf[off + 6 + i] = b;
        sum += b;
        add += sum;
    }
    buf[off + 6 + len] = sum;
    buf[off + 7 + len] = add;

    return len + 8;
}

void AnoComProtocol::receiveData()
{
    // 限制单轮处理字节数, 防止地面站持续发送时阻塞主循环。
    // 256 字节 ≈ 1 个最大帧 (ANO_MAX_DATA_LEN=256 + 8 帧开销), 足以处理单帧。
    const uint16_t MAX_RX_BYTES_PER_CALL = 256;
    uint16_t bytes_processed = 0;
    while (_serial->available() && bytes_processed < MAX_RX_BYTES_PER_CALL)
    {
        bytes_processed++;
        uint8_t data = _serial->read();

        if (_rxIndex == 0 && data != ANO_FRAME_HEAD)
        {
            continue; // 寻找帧头
        }

        _rxBuffer[_rxIndex++] = data;

        if (_rxIndex >= 8 && _rxIndex == (_rxBuffer[4] + (_rxBuffer[5] << 8)) + 8)
        {
            // 数据接收完成
            _dataReceived = true;
            _rxIndex = 0;
            parseData(_rxBuffer, _rxBuffer[4] + (_rxBuffer[5] << 8) + 8);
        }

        if (_rxIndex >= ANO_MAX_DATA_LEN + 8)
        {
            // 超过最大长度，重置接收
            _rxIndex = 0;
        }
    }
}

uint8_t AnoComProtocol::calculateSumCheck(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

uint8_t AnoComProtocol::calculateAddCheck(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    uint8_t add = 0;
    for (int i = 0; i < len; i++)
    {
        sum += data[i];
        add += sum;
    }
    return add;
}

void AnoComProtocol::parseData(uint8_t *data, uint16_t len)
{
    if (data[0] != ANO_FRAME_HEAD)
        return;

    uint8_t sumCheck = calculateSumCheck(data, len - 2);
    uint8_t addCheck = calculateAddCheck(data, len - 2);

    if (sumCheck != data[len - 2] || addCheck != data[len - 1])
    {
        return; // 校验失败
    }

    // 数据解析
    uint8_t funcCode = data[3];
    // 保存源帧的校验值，供安全协议回传使用（0xE0/0xE1 参数帧 → 0x00 校验帧）
    uint8_t rxSumCheck = data[len - 2];
    uint8_t rxAddCheck = data[len - 1];

    // 通用上行回调：通信模块可在此处理任意功能码
    if (g_ano_rx_callback)
    {
        g_ano_rx_callback(funcCode, &data[6], len - 8, rxSumCheck, rxAddCheck);
    }

    switch (funcCode)
    {
    case ANO_FUNC_DATA_CHECK:
        if (_onDataCheckCallback)
        {
            _onDataCheckCallback(data[6], data[7], data[8]);
        }
        break;
    // ... 其他功能码的解析
    default:
        break;
    }
}

/**
 * @brief 发送惯性传感器数据 (ID: 0x01)
 *
 * @param accX 加速度 X轴, 单位: m/s²
 * @param accY 加速度 Y轴, 单位: m/s²
 * @param accZ 加速度 Z轴, 单位: m/s²
 * @param gyrX 陀螺仪 X轴, 单位: deg/s (度/秒)
 * @param gyrY 陀螺仪 Y轴, 单位: deg/s (度/秒)
 * @param gyrZ 陀螺仪 Z轴, 单位: deg/s (度/秒)
 * @param shockSta 震动状态 (0:无震动, 1-255:震动强度等级)
 */
void AnoComProtocol::sendIMUData(float accX, float accY, float accZ, float gyrX, float gyrY, float gyrZ, uint8_t shockSta)
{
    // -----------------------------------------------------------------
    // 1. 加速度处理 (ACC)
    // -----------------------------------------------------------------
    // 协议单位: cm/s²
    // 假设输入单位为 m/s² (1 m/s² = 100 cm/s²)
    int32_t send_acc_x = (int32_t)(accX * 100.0f);
    int32_t send_acc_y = (int32_t)(accY * 100.0f);
    int32_t send_acc_z = (int32_t)(accZ * 100.0f);

    // -----------------------------------------------------------------
    // 2. 陀螺仪处理 (GYR) - 关键修正点
    // -----------------------------------------------------------------
    // 协议定义: 32768 对应 2000 deg/s
    // 转换系数: 32768 / 2000 = 16.384
    const float GYRO_SCALE_FACTOR = 16.384f;

    int32_t temp_gyr_x = (int32_t)(gyrX * GYRO_SCALE_FACTOR);
    int32_t temp_gyr_y = (int32_t)(gyrY * GYRO_SCALE_FACTOR);
    int32_t temp_gyr_z = (int32_t)(gyrZ * GYRO_SCALE_FACTOR);

    // 必须限幅！否则超过 2000度/秒 会导致 int16 溢出翻转
    int16_t send_gyr_x = (int16_t)constrain(temp_gyr_x, -32767, 32767);
    int16_t send_gyr_y = (int16_t)constrain(temp_gyr_y, -32767, 32767);
    int16_t send_gyr_z = (int16_t)constrain(temp_gyr_z, -32767, 32767);

    // -----------------------------------------------------------------
    // 3. 数据打包 (Little Endian)
    // -----------------------------------------------------------------
    uint8_t data[13];

    // ACC
    data[0] = (uint8_t)(send_acc_x & 0xFF);
    data[1] = (uint8_t)((send_acc_x >> 8) & 0xFF);
    data[2] = (uint8_t)(send_acc_y & 0xFF);
    data[3] = (uint8_t)((send_acc_y >> 8) & 0xFF);
    data[4] = (uint8_t)(send_acc_z & 0xFF);
    data[5] = (uint8_t)((send_acc_z >> 8) & 0xFF);

    // GYR
    data[6] = (uint8_t)(send_gyr_x & 0xFF);
    data[7] = (uint8_t)((send_gyr_x >> 8) & 0xFF);
    data[8] = (uint8_t)(send_gyr_y & 0xFF);
    data[9] = (uint8_t)((send_gyr_y >> 8) & 0xFF);
    data[10] = (uint8_t)(send_gyr_z & 0xFF);
    data[11] = (uint8_t)((send_gyr_z >> 8) & 0xFF);

    // SHOCK
    data[12] = shockSta;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_IMU_DATA, data, 13);
}

void AnoComProtocol::sendMagPressTempData(int16_t magX, int16_t magY, int16_t magZ, float temp, uint8_t magSta)
{
    // 温度放大10倍并转换为 int16_t
    int16_t tempInt = static_cast<int16_t>(temp * 10);

    uint8_t data[9];
    data[0] = magX & 0xFF;
    data[1] = (magX >> 8) & 0xFF;
    data[2] = magY & 0xFF;
    data[3] = (magY >> 8) & 0xFF;
    data[4] = magZ & 0xFF;
    data[5] = (magZ >> 8) & 0xFF;
    data[6] = tempInt & 0xFF;
    data[7] = (tempInt >> 8) & 0xFF;
    data[8] = magSta;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_MAG_PRESS_TEMP_DATA, data, 9);
}

void AnoComProtocol::sendAttitudeEuler(float roll, float pitch, float yaw, uint8_t fusionStatus)
{
    // 放大100倍并转换为 int16_t
    int16_t rollScaled = (int16_t)(roll * 100);
    int16_t pitchScaled = (int16_t)(pitch * 100);
    int16_t yawScaled = (int16_t)(yaw * 100);

    uint8_t data[7];
    data[0] = rollScaled & 0xFF;
    data[1] = (rollScaled >> 8) & 0xFF;
    data[2] = pitchScaled & 0xFF;
    data[3] = (pitchScaled >> 8) & 0xFF;
    data[4] = yawScaled & 0xFF;
    data[5] = (yawScaled >> 8) & 0xFF;
    data[6] = fusionStatus;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_ATTITUDE_EULER, data, 7);
}

void AnoComProtocol::sendAttitudeQuat(float v0, float v1, float v2, float v3, uint8_t fusionSta)
{
    // 放大10000倍并转换为 int16_t
    int16_t q0 = (int16_t)(v0 * 10000);
    int16_t q1 = (int16_t)(v1 * 10000);
    int16_t q2 = (int16_t)(v2 * 10000);
    int16_t q3 = (int16_t)(v3 * 10000);

    uint8_t data[9];
    data[0] = q0 & 0xFF;
    data[1] = (q0 >> 8) & 0xFF;
    data[2] = q1 & 0xFF;
    data[3] = (q1 >> 8) & 0xFF;
    data[4] = q2 & 0xFF;
    data[5] = (q2 >> 8) & 0xFF;
    data[6] = q3 & 0xFF;
    data[7] = (q3 >> 8) & 0xFF;
    data[8] = fusionSta;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_ATTITUDE_QUAT, data, 9);
}

void AnoComProtocol::sendAltitudeData(float altBar, float altAdd, float altFu, uint8_t altSta)
{
    int32_t altBarInt = static_cast<int32_t>(altBar * 100);
    int32_t altAddInt = static_cast<int32_t>(altAdd * 100);
    int32_t altFuInt = static_cast<int32_t>(altFu * 100);

    uint8_t data[13];
    data[0] = altBarInt & 0xFF;
    data[1] = (altBarInt >> 8) & 0xFF;
    data[2] = (altBarInt >> 16) & 0xFF;
    data[3] = (altBarInt >> 24) & 0xFF;
    data[4] = altAddInt & 0xFF;
    data[5] = (altAddInt >> 8) & 0xFF;
    data[6] = (altAddInt >> 16) & 0xFF;
    data[7] = (altAddInt >> 24) & 0xFF;
    data[8] = altFuInt & 0xFF;
    data[9] = (altFuInt >> 8) & 0xFF;
    data[10] = (altFuInt >> 16) & 0xFF;
    data[11] = (altFuInt >> 24) & 0xFF;
    data[12] = altSta;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_ALTITUDE_DATA, data, 13);
}

void AnoComProtocol::sendFlightMode(uint8_t mode, uint8_t sFlag, uint8_t cId, uint8_t cmd0, uint8_t cmd1)
{
    uint8_t data[5];
    data[0] = mode;
    data[1] = sFlag;
    data[2] = cId;
    data[3] = cmd0;
    data[4] = cmd1;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_FLIGHT_MODE, data, 5);
}

void AnoComProtocol::sendFlightSpeed(int16_t speedX, int16_t speedY, int16_t speedZ)
{
    uint8_t data[6];
    data[0] = speedX & 0xFF;
    data[1] = (speedX >> 8) & 0xFF;
    data[2] = speedY & 0xFF;
    data[3] = (speedY >> 8) & 0xFF;
    data[4] = speedZ & 0xFF;
    data[5] = (speedZ >> 8) & 0xFF;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_FLIGHT_SPEED, data, 6);
}

void AnoComProtocol::sendPosOffset(int32_t posX, int32_t posY, int32_t posZ)
{
    uint8_t data[12];
    data[0] = posX & 0xFF;
    data[1] = (posX >> 8) & 0xFF;
    data[2] = (posX >> 16) & 0xFF;
    data[3] = (posX >> 24) & 0xFF;
    data[4] = posY & 0xFF;
    data[5] = (posY >> 8) & 0xFF;
    data[6] = (posY >> 16) & 0xFF;
    data[7] = (posY >> 24) & 0xFF;
    data[8] = posZ & 0xFF;
    data[9] = (posZ >> 8) & 0xFF;
    data[10] = (posZ >> 16) & 0xFF;
    data[11] = (posZ >> 24) & 0xFF;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_POS_OFFSET, data, 12);
}

void AnoComProtocol::sendWindEstimate(int16_t windX, int16_t windY)
{
    uint8_t data[4];
    data[0] = windX & 0xFF;
    data[1] = (windX >> 8) & 0xFF;
    data[2] = windY & 0xFF;
    data[3] = (windY >> 8) & 0xFF;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_WIND_EST, data, 4);
}

void AnoComProtocol::sendTargetAttitude(int16_t tarRol, int16_t tarPit, int16_t tarYaw)
{
    uint8_t data[6];
    data[0] = tarRol & 0xFF;
    data[1] = (tarRol >> 8) & 0xFF;
    data[2] = tarPit & 0xFF;
    data[3] = (tarPit >> 8) & 0xFF;
    data[4] = tarYaw & 0xFF;
    data[5] = (tarYaw >> 8) & 0xFF;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_TARGET_ATTITUDE, data, 6);
}

void AnoComProtocol::sendTargetSpeed(int16_t tarSpeedX, int16_t tarSpeedY, int16_t tarSpeedZ)
{
    uint8_t data[6];
    data[0] = tarSpeedX & 0xFF;
    data[1] = (tarSpeedX >> 8) & 0xFF;
    data[2] = tarSpeedY & 0xFF;
    data[3] = (tarSpeedY >> 8) & 0xFF;
    data[4] = tarSpeedZ & 0xFF;
    data[5] = (tarSpeedZ >> 8) & 0xFF;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_TARGET_SPEED, data, 6);
}

void AnoComProtocol::sendReturnInfo(int16_t ra, uint16_t rd)
{
    uint8_t data[4];
    data[0] = ra & 0xFF;
    data[1] = (ra >> 8) & 0xFF;
    data[2] = rd & 0xFF;
    data[3] = (rd >> 8) & 0xFF;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_RETURN_INFO, data, 4);
}

/**
 * @brief 发送电压电流数据实现
 * @note 手册中表格写 BAT_VOTAGE*1000，但下方文字说明写“扩大100倍”。
 *       此处遵循文字说明采用 *100 (0.01V精度)。如果上位机显示电压偏小10倍，请改为 *1000。
 */
void AnoComProtocol::sendVoltCurr(float bat_voltage, float bat_current, float fc_voltage, float fc_current, uint16_t power_state_flags)
{
    // 1. 数据转换与缩放
    // --------------------------------------------------------
    // 注意：强制类型转换前建议进行限幅，防止溢出，此处为简化逻辑直接转换
    uint16_t u16_bat_v = (uint16_t)(bat_voltage * 100.0f);
    uint16_t u16_bat_c = (uint16_t)(bat_current * 100.0f);
    uint16_t u16_fc_v = (uint16_t)(fc_voltage * 100.0f);
    uint16_t u16_fc_c = (uint16_t)(fc_current * 100.0f);

    // 2. 数据打包 (Little Endian / 小端模式)
    // --------------------------------------------------------
    // 协议要求 Data 区域长度为 5 * U16 = 10 Bytes
    uint8_t data[10];

    // 电池电压 (Bat Voltage)
    data[0] = (uint8_t)(u16_bat_v & 0xFF);        // 低8位
    data[1] = (uint8_t)((u16_bat_v >> 8) & 0xFF); // 高8位

    // 电池电流 (Bat Current)
    data[2] = (uint8_t)(u16_bat_c & 0xFF);
    data[3] = (uint8_t)((u16_bat_c >> 8) & 0xFF);

    // 飞控电压 (FC Voltage)
    data[4] = (uint8_t)(u16_fc_v & 0xFF);
    data[5] = (uint8_t)((u16_fc_v >> 8) & 0xFF);

    // 飞控电流 (FC Current)
    data[6] = (uint8_t)(u16_fc_c & 0xFF);
    data[7] = (uint8_t)((u16_fc_c >> 8) & 0xFF);

    // 电源状态 (直接填入组合好的标志位)
    data[8] = (uint8_t)(power_state_flags & 0xFF);
    data[9] = (uint8_t)((power_state_flags >> 8) & 0xFF);

    // 3. 发送数据
    // --------------------------------------------------------
    // ID: 0x0D (ANO_FUNC_VOLT_CURR), Length: 10
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_VOLT_CURR, data, 10);
}

void AnoComProtocol::sendExtModuleStatus(uint8_t staGVel, uint8_t staGPos, uint8_t staGps, uint8_t staAltAdd)
{
    uint8_t data[4];
    data[0] = staGVel;
    data[1] = staGPos;
    data[2] = staGps;
    data[3] = staAltAdd;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_EXT_MODULE_STATUS, data, 4);
}

void AnoComProtocol::sendRGBOutput(uint8_t briR, uint8_t briG, uint8_t briB, uint8_t briA)
{
    uint8_t data[4];
    data[0] = briR;
    data[1] = briG;
    data[2] = briB;
    data[3] = briA;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_RGB_OUTPUT, data, 4);
}

void AnoComProtocol::sendPWMOutput(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4, uint16_t pwm5, uint16_t pwm6, uint16_t pwm7, uint16_t pwm8)
{
    uint8_t data[16];
    data[0] = pwm1 & 0xFF;
    data[1] = (pwm1 >> 8) & 0xFF;
    data[2] = pwm2 & 0xFF;
    data[3] = (pwm2 >> 8) & 0xFF;
    data[4] = pwm3 & 0xFF;
    data[5] = (pwm3 >> 8) & 0xFF;
    data[6] = pwm4 & 0xFF;
    data[7] = (pwm4 >> 8) & 0xFF;
    data[8] = pwm5 & 0xFF;
    data[9] = (pwm5 >> 8) & 0xFF;
    data[10] = pwm6 & 0xFF;
    data[11] = (pwm6 >> 8) & 0xFF;
    data[12] = pwm7 & 0xFF;
    data[13] = (pwm7 >> 8) & 0xFF;
    data[14] = pwm8 & 0xFF;
    data[15] = (pwm8 >> 8) & 0xFF;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_PWM_OUTPUT, data, 16);
}

/**
 * @brief 发送遥控器数据 (ID: 0x40, 手册定义)
 *
 * 手册 0x40：ROL/PIT/THR/YAW/AUX1-6 共 10×int16，范围 1000-2000 us。
 * 2026-08-10 数据归位：0x40 恢复手册遥控帧（原本工程自定义执行器帧迁至 0xF1）。
 */
void AnoComProtocol::sendRCData(const uint16_t *rc)
{
    uint8_t data[20];
    for (int i = 0; i < 10; i++)
    {
        uint16_t v = rc ? rc[i] : 0;
        data[i * 2] = v & 0xFF;
        data[i * 2 + 1] = (v >> 8) & 0xFF;
    }
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_RC_DATA, data, 20);
}

void AnoComProtocol::sendAttitudeControl(int16_t ctrlRol, int16_t ctrlPit, int16_t ctrlThr, int16_t ctrlYaw)
{
    uint8_t data[8];
    data[0] = ctrlRol & 0xFF;
    data[1] = (ctrlRol >> 8) & 0xFF;
    data[2] = ctrlPit & 0xFF;
    data[3] = (ctrlPit >> 8) & 0xFF;
    data[4] = ctrlThr & 0xFF;
    data[5] = (ctrlThr >> 8) & 0xFF;
    data[6] = ctrlYaw & 0xFF;
    data[7] = (ctrlYaw >> 8) & 0xFF;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_ATTITUDE_CONTROL, data, 8);
}

void AnoComProtocol::sendGPSInfo1(uint8_t fixSta, uint8_t sNum, float lng, float lat, float altGps, float nSpe, float eSpe, float dSpe, float pdop, float vacc, float sacc)
{
    int32_t lngInt = static_cast<int32_t>(lng * 10000000);  // 放大10000000倍并转换为 int32_t
    int32_t latInt = static_cast<int32_t>(lat * 10000000);  // 放大10000000倍并转换为 int32_t
    int32_t altGpsInt = static_cast<int32_t>(altGps * 100); // 放大100倍并转换为 int32_t
    int16_t nSpeInt = static_cast<int16_t>(nSpe * 100);     // 放大100倍并转换为 int16_t
    int16_t eSpeInt = static_cast<int16_t>(eSpe * 100);     // 放大100倍并转换为 int16_t
    int16_t dSpeInt = static_cast<int16_t>(dSpe * 100);     // 放大100倍并转换为 int16_t
    uint8_t pdopInt = static_cast<uint8_t>(pdop * 10);
    uint8_t vaccInt = static_cast<uint8_t>(vacc * 10);
    uint8_t saccInt = static_cast<uint8_t>(sacc * 10);

    uint8_t data[23];
    data[0] = fixSta;
    data[1] = sNum;
    data[2] = lngInt & 0xFF;
    data[3] = (lngInt >> 8) & 0xFF;
    data[4] = (lngInt >> 16) & 0xFF;
    data[5] = (lngInt >> 24) & 0xFF;
    data[6] = latInt & 0xFF;
    data[7] = (latInt >> 8) & 0xFF;
    data[8] = (latInt >> 16) & 0xFF;
    data[9] = (latInt >> 24) & 0xFF;
    data[10] = altGpsInt & 0xFF;
    data[11] = (altGpsInt >> 8) & 0xFF;
    data[12] = (altGpsInt >> 16) & 0xFF;
    data[13] = (altGpsInt >> 24) & 0xFF;
    data[14] = nSpeInt & 0xFF;
    data[15] = (nSpeInt >> 8) & 0xFF;
    data[16] = eSpeInt & 0xFF;
    data[17] = (eSpeInt >> 8) & 0xFF;
    data[18] = dSpeInt & 0xFF;
    data[19] = (dSpeInt >> 8) & 0xFF;
    data[20] = pdopInt;
    data[21] = vaccInt;
    data[22] = saccInt;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_GPS_INFO1, data, 23);
}

// ... 其他数据发送函数的实现

void AnoComProtocol::addFlexData(uint8_t frameId, AnoFlexDataInfo dataInfo)
{
    if (frameId >= ANO_FUNC_FLEX_DATA_START && frameId <= ANO_FUNC_FLEX_DATA_END && _flexDataCount[frameId - ANO_FUNC_FLEX_DATA_START] < 5)
    {
        _flexData[frameId - ANO_FUNC_FLEX_DATA_START][_flexDataCount[frameId - ANO_FUNC_FLEX_DATA_START]] = dataInfo;
        _flexDataCount[frameId - ANO_FUNC_FLEX_DATA_START]++;
    }
}

void AnoComProtocol::sendFlexData(uint8_t frameId)
{
    if (frameId >= ANO_FUNC_FLEX_DATA_START && frameId <= ANO_FUNC_FLEX_DATA_END)
    {
        uint8_t data[ANO_MAX_DATA_LEN];
        uint8_t index = 0;
        for (int i = 0; i < _flexDataCount[frameId - ANO_FUNC_FLEX_DATA_START]; i++)
        {
            // 根据数据类型获取数据，并添加到data数组中
            switch (_flexData[frameId - ANO_FUNC_FLEX_DATA_START][i].type)
            {
            case ANO_UINT8:
                // 假设通过某个函数或变量获取数据，例如 getUint8Data(dataInfo.id)
                // data[index++] = getUint8Data(_flexData[frameId - ANO_FUNC_FLEX_DATA_START][i].id);
                break;

            case ANO_INT16:
                // data[index++] = getInt16Data(_flexData[frameId - ANO_FUNC_FLEX_DATA_START][i].id) & 0xFF;
                // data[index++] = (getInt16Data(_flexData[frameId - ANO_FUNC_FLEX_DATA_START][i].id) >> 8) & 0xFF;
                break;
            // ... 其他数据类型
            default:
                break;
            }
        }
        sendData(ANO_GND_STATION_ADDR, frameId, data, index);
    }
}

void AnoComProtocol::onDataCheck(void (*callback)(uint8_t idGet, uint8_t scGet, uint8_t acGet))
{
    _onDataCheckCallback = callback;
}

// ... 其他回调函数注册的实现

void AnoComProtocol::sendDataCheck(uint8_t idGet, uint8_t scGet, uint8_t acGet)
{
    uint8_t data[3];
    data[0] = idGet;
    data[1] = scGet;
    data[2] = acGet;
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_DATA_CHECK, data, 3);
}

void AnoComProtocol::sendDeviceInfo(uint8_t devId, int16_t hwVer, int16_t swVer,
                                    int16_t blVer, int16_t ptVer, const char *devName)
{
    // 协议 0xE3 DATA: DEV_ID(U8) + HW_VER(I16) + SW_VER(I16) + BL_VER(I16) + PT_VER(I16) + DEV_NAME(char*N)
    uint8_t data[1 + 8 + 20];
    uint8_t idx = 0;
    data[idx++] = devId;
    // HW_VER
    data[idx++] = (uint8_t)(hwVer & 0xFF);
    data[idx++] = (uint8_t)((hwVer >> 8) & 0xFF);
    // SW_VER
    data[idx++] = (uint8_t)(swVer & 0xFF);
    data[idx++] = (uint8_t)((swVer >> 8) & 0xFF);
    // BL_VER
    data[idx++] = (uint8_t)(blVer & 0xFF);
    data[idx++] = (uint8_t)((blVer >> 8) & 0xFF);
    // PT_VER
    data[idx++] = (uint8_t)(ptVer & 0xFF);
    data[idx++] = (uint8_t)((ptVer >> 8) & 0xFF);
    // DEV_NAME: 最多 20 字节, 不足补 0x00
    uint8_t nameLen = 0;
    while (devName && devName[nameLen] && nameLen < 20)
    {
        data[idx++] = (uint8_t)devName[nameLen++];
    }
    while (nameLen++ < 20)
    {
        data[idx++] = 0x00;
    }
    sendData(ANO_GND_STATION_ADDR, ANO_FUNC_DEVICE_INFO, data, idx);
}
void AnoComProtocol::setRxCallback(void (*cb)(uint8_t, uint8_t *, uint16_t, uint8_t, uint8_t))
{
    g_ano_rx_callback = cb;
}
