# CC-AEGT 参考库（仅测试）

来源：外部实飞工程 `D:\PIO_Projects\VTVL_LiquidRocket_FC\lib\CCAEGT_Library`。

- `CCAEGT.cpp/.h`、`CCAEGT_Config.h`、`CCAEGT_Types.h` 逐字复制，仅补充 GOV-004 要求的 `@module` 注释字段；
- 仅供 `test_host/test_liquid_ccaegt_reference.cpp` 做“实飞参考制导 + 本机推进模型”的组合 SIL；
- **未接入本仓库固件飞行模式**，也不随固件编译；
- 已知参数不一致：CCAEGT `M_DRY=12.5 kg`，外部固件 `DRY_MASS=13.5 kg`；参考测试按 CCAEGT 的 16.5 kg 总量对齐，不修改外部飞行固件。
