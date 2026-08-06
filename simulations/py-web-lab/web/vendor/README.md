# vendor — 第三方浏览器依赖

| 组件 | 版本 | 许可证 | 用途 |
|---|---|---|---|
| Three.js | r170 | MIT | WebGL 渲染、数学库、几何体 |
| es-module-shims | 1.10.0 | MIT | import map 兼容垫片 |

- `three-r170/three.module.js`：Three.js 主模块（文件头含 MIT 许可与 REVISION 标记）
- `three-r170/addons/`：OrbitControls 与后处理模块（当前运行闭包仅使用 EffectComposer/RenderPass/UnrealBloomPass 及其内部依赖；其余模块保留供扩展，部分未使用模块的配套 shader 未随附，不能直接引用）
- `three-r170/es-module-shims.min.js`：ES Module Shim

第三方代码不属于本项目自研源码；升级时记录版本、来源与校验值。
