# Display Drivers

此分类暂未声明通用显示 core。既有 OLED 实现已整理到 `ports/ch32/legacy/display/oled/`，它仍依赖固定 HAL 与管脚配置。

后续迁移必须先抽出显示传输、复位和延时 port，再建立独立 `components/drivers/display/<device>/` 组件；请勿直接将 legacy 目录作为通用库使用。
