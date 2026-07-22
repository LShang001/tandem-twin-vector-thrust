# 飞行器构型 AI 图候选（2026-07-23）

本目录保存未被采用也不应丢弃的 AI 生成候选。它们是视觉概念资产，不能替代 Web 仿真模型、`models/aircraft-model.json` 或工程图。

## 生成信息

- 生成日期：2026-07-23
- 生成方式：AI 图像生成工具，参考图驱动生成
- 权威参考图：`docs/03-理论推导/THY-004/fig/01-web-render.png`
- 关键构型约束：参考视角中左侧为前发拉力式，右侧为尾发推进式；恰好两副纵列螺旋桨；不得形成 X 形机翼歧义

## 文件与视觉审查

| 文件 | 定位 | 当前审查结论 | SHA-256 |
|---|---|---|---|
| `configuration-candidate-a-cad.png` | 白底 CAD 产品渲染 | 可作为概念展示候选；机体与推进器辨识度较高，但尾翼、翼尖小翼和摆轴细节未经工程验证 | `CE56CD802BBEA354E0ADB6B3B462C5E274B963A792057FBC166F1A6B7C589F2E` |
| `configuration-candidate-b-technical.png` | 简洁技术插图 | 轮廓清楚、适合后续标注；机翼与尾翼拓扑、摆轴细节仍有 AI 自由发挥，不得直接作为论文工程构型图 | `8C81C27D4D571894E7EC3DDA294FE542A831C3993EF8DEEF0FC9BBE914459A0E` |
| `configuration-candidate-c-industrial-cad.png` | 工业 CAD 渲染 | 前后发正确、留白充足，但前后桨叶外形不一致且摆座同色，差速反扭语义偏弱 | `D2C9DAFF93A789BB0B8837773623D175C008AC3C0C18299B3742627E9C6F94E7` |
| `configuration-candidate-d-textbook.png` | 教材式技术插图 | C/D/E 中缩印与轮廓最佳，前蓝尾橙区分清楚；环架仍可能被误读为双轴万向节 | `AC3E4117790C59BA87AF17F8746E7B0B2EA5E376CCC6C9D17236CA3B6CE22257` |
| `configuration-candidate-e-cutaway.png` | 机械剖视概念图 | 视觉冲击强，但虚构轴承、绕组和支架细节，容易夸大当前概念级成熟度，仅保留作展示候选 | `B8935CA962198ABA7E920EA6086AA01ECE169E0EDE2066517208232447C71A51` |
| `configuration-candidate-f-single-axis.png` | 单轴摆座强化版 | 原论文底图基线；三分之四透视较强，单主翼拓扑仍需说明 | `CE8C907321D83E2AE632C782DE85E283BA9ADAE733C2279F4CB747811031A0DC` |
| `configuration-candidate-g-clarified-cad.png` | 结构澄清 CAD 编辑版 | 前后发和摆座清楚，但近侧翼面过重、远侧翼尖孤立，缩印没有超越 F | `DCF673DDD1D42A8A90958E05264CF3F539D2088F12AA006D5D73525D2941F614` |
| `configuration-candidate-h-textbook-flat.png` | 教材式扁平插图 | 留白和标注适配性均衡；近侧翼尖小翼较像端板，单翼语义略弱于 J | `1C14A8F7427459EA9038E67157816805263BD169CF6DAA8BA74DAA42DED5B6C6B` |
| `configuration-candidate-i-high-contrast.png` | 高对比深色版 | 彩色屏幕醒目，但灰度打印会吞掉机身、座舱和尾摆座细节，明确淘汰 | `858A5549F09DA133F769B8E497075D160B02EB27C3E1E0D86AA7D300BE651AA6` |
| `configuration-candidate-j-near-vector.png` | 近矢量技术线稿 | 新一轮首选；单主翼、两侧翼尖小翼、缩印和灰度最稳，仍需 SVG 明确 `z_b/y_b` | `8746FE0FA73537C6E2A5E753B288F13B4F34E0FCD8DDB008E8A9FA464B53D9C8` |
| `configuration-candidate-k-oblique-vector.png` | 斜视正交轴强化版 | 翼面和前 `z_b` 很清楚，但前挂架像隔框、尾部多圆点像多连杆，不替代 J | `11DF94E6FD682A28639A6BAE25755ED40E9755A0AED5D8371EE2CEEB86BCE9B0` |
| `configuration-candidate-l-counter-rotating-props.png` | 反向桨手性修正版 | 尝试只修正前后桨手性，但 AI 同时改变相机与尾挂架，静态叶型仍不足以证明旋向；不替代 J | `00044899F78A4131B590B97261D7BD287770D8252D1D43517FD33867FA06C8D2` |
| `configuration-candidate-m-orthogonal-deflection.png` | 非零摆角编辑候选 | M/N 中更清楚地表现两台电机轴不共线；三路审查均判定仍不能由裸图证明前绕 `z_b`、尾绕 `y_b`，仅保留作 AI 概念候选 | `08FD0E8D076ECE9757BFD5E43AA5FB9D9B7E5BC2478C49BD14CAEF6B3DB2036A` |
| `configuration-candidate-n-explicit-orthogonal-prompt.png` | 显式正交提示词候选 | 提示词约束更强，但尾发偏转和桨盘刚体关系反而更弱；独立审查一致判定不优于 M | `13FD8C6F5E2BE883781A87920E5D19A0BE3F669A4879E47EBD752084B33A860F` |
| `comparison-f-j.png` | F-J 统一尺寸盲比板 | 仅用于候选审查，不进入论文正文 | `1750488FAB4F18120190D6A240BA82C88E675F9E6B1930EF6B933C14194EFE00` |

候选 A-L 为前序 AI 图像通道输出；G 是对 F 的精确编辑，H-J 是独立风格候选，K/L 是对 J 的定向编辑。
M/N 使用文件级参考图编辑通道 `mcp__uTools__utools_zfc4py1y_image_generate`，尺寸均为 `1264 x 848`；M 参考 J，N 参考 M。
三路独立审查结果如下：早期视觉审查 `J > K > H > F > G > I`；新一轮非零摆角审查一致为 `M > N`，但 M/N 裸图都无法证明两摆轴正交。
因此论文图 1 不再直接采用 AI 底图，而改用 Web `aircraft-view.mjs` 的确定性非零摆角渲染；全部 AI 候选继续留档，不删除。

## 候选 G 提示词

```text
Use case: precise-object-edit
Asset type: traceable candidate G for a graduate engineering LaTeX report figure
Input images: Image 1 is the Web simulation screenshot and is the authoritative geometry/orientation reference. Image 2 is the current selected white-background candidate F and is the edit target.
Primary request: improve only the engineering readability of candidate F while preserving its camera and overall airframe. Make the aircraft topology unmistakable at first glance.
Critical orientation invariant: the LEFT side is the aircraft FRONT/nose and +x_b direction. The LEFT propulsor is the FRONT tractor unit, with the propeller ahead of the motor. The RIGHT side is the aircraft REAR/tail. The RIGHT propulsor is the TAIL pusher unit, with the propeller behind the motor. Never swap them.
Required geometry: exactly two longitudinally aligned propellers; one continuous conventional main wing with a left and right half; two small wingtip fins at the two outer wing tips; no separate horizontal tailplane; slender fuselage.
Front mount on LEFT: one simple cyan single-axis yaw hinge about vertical body z_b, shown as a clean top-and-bottom vertical trunnion/yoke, not a ring cage and not a universal joint.
Tail mount on RIGHT: one simple orange single-axis pitch hinge about transverse body y_b, shown as a clean left-and-right horizontal trunnion/yoke, not a ring cage and not a universal joint.
Propellers: two matching three-blade propellers with consistent design and scale, fully visible.
Style/medium: premium aerospace patent illustration blended with restrained CAD line-and-wash rendering; crisp dark outlines, matte white and light-gray aircraft surfaces, minimal reflections, high local contrast, no photorealistic clutter.
Composition/framing: landscape 3:2, same three-quarter top view as candidate F, aircraft fills about 78 percent of canvas, wide clean white margin around both propellers for later deterministic HTML/SVG annotations.
Constraints: change only wing topology clarity, matching propellers, and the two single-axis mounts; preserve front-left/rear-right ordering, fuselage silhouette, camera, lighting, and clean background. No text, no letters, no arrows, no axis triad, no numbers, no watermark, no extra rotors, no extra wings, no X-wing, no cockpit emphasis, no fictional mechanical internals.
```

## 候选 H 提示词

```text
Use case: scientific-educational
Asset type: traceable candidate H background for a graduate engineering LaTeX report; all labels will be added later with deterministic HTML/SVG
Input images: Image 1 is the Web simulation screenshot and is the authoritative configuration/orientation reference. Image 2 is the current candidate F only as a rough composition reference, not as engineering authority.
Primary request: create a highly legible modern engineering textbook illustration of the same concept aircraft, optimized for reduction to approximately 150 mm width in a PDF.
Critical orientation invariant: aircraft nose and +x_b point to the LEFT. LEFT is the FRONT tractor propulsor, with its propeller ahead of the motor. RIGHT is the REAR tail pusher propulsor, with its propeller behind the motor. Never reverse this order.
Required aircraft architecture: exactly two inline propellers on the fuselage axis; one conventional continuous main wing with left and right halves; small vertical fins only at the two outer wing tips; no separate horizontal tailplane; no second wing; slender central fuselage.
Mechanisms: front LEFT mount is one cyan yaw hinge about vertical z_b, depicted by a simple vertical top-and-bottom clevis/trunnion. Rear RIGHT mount is one orange pitch hinge about transverse body y_b, depicted by a simple horizontal left-and-right clevis/trunnion. Each must read as single-axis, not universal or spherical.
Style/medium: flat-shaded axonometric patent/textbook illustration, subtle thin dark contour lines, restrained two-tone gray airframe, crisp black propellers, cyan front hardware and orange rear hardware, no glossy reflections and no fabricated internal details.
Composition/framing: wide 3:2 landscape, stable near-isometric three-quarter top view, aircraft centered and fills 72 to 78 percent of the frame, both propellers fully visible, generous white annotation space around left, right, and above the fuselage.
Readability constraints: strong silhouette; foreground and far wing halves clearly belong to the same single wing; front and rear mechanisms have distinct shapes as well as colors; matching three-blade propellers.
Constraints: no text, no labels, no arrows, no axis triad, no numbers, no watermark, no shadow darker than very light gray, no scenery, no X-wing, no extra rotors, no duplicated wing, no tailplane, no cockpit canopy emphasis, no military details, no exploded components.
```

## 候选 I 提示词

```text
Use case: scientific-educational
Asset type: traceable candidate I, high-contrast base illustration for a graduate engineering LaTeX report
Input images: Image 1 is the Web simulation screenshot and defines the authoritative aircraft architecture and orientation. Image 2 is the current white candidate F only as a camera/framing reference.
Primary request: produce a cleaner high-contrast aerospace systems illustration whose airframe topology remains readable in grayscale and at small PDF size.
Critical orientation invariant: nose and +x_b point LEFT. The LEFT propulsor is the FRONT tractor motor with propeller ahead of nacelle. The RIGHT propulsor is the REAR pusher motor with propeller behind nacelle. Do not reverse them.
Architecture: exactly two inline propulsors; slender single fuselage; one continuous main wing with two halves; small vertical fins at the two outer wing tips only; no independent horizontal tailplane and no second wing.
Mechanisms: LEFT/front mount is a cyan single-axis vertical yaw clevis about z_b. RIGHT/rear mount is an orange single-axis transverse pitch clevis about y_b. Avoid all circular cage or universal-joint imagery.
Style/medium: refined technical cutaway-free CAD illustration with simplified hard surfaces; dark graphite fuselage, light aluminum main wing, near-black motors and matching three-blade propellers, cyan front clevis, orange rear clevis. Crisp outlines, restrained ambient occlusion, no glossy highlights, no fake fasteners or internal components.
Composition/framing: landscape 3:2, near-isometric three-quarter top view close to Image 2, aircraft fills around 75 percent of canvas, both propellers and both mounts unobstructed, clean white background with only a very faint grounding shadow and ample annotation space.
Perceptual requirements: silhouette is immediately readable; main wing halves visibly connect through the fuselage; front and rear propulsors have equal visual weight; cyan and orange remain distinguishable by both shape and luminance; no unnecessary canopy or decorative panels.
Constraints: no text, no letters, no arrows, no labels, no axis triad, no numbers, no watermark; no X-wing; no extra rotors; no extra fins; no tailplane; no cockpit emphasis; no military equipment; no scenery; no dramatic perspective distortion.
```

## 候选 J 提示词

```text
Use case: scientific-educational
Asset type: traceable candidate J, near-vector technical line illustration for a graduate engineering LaTeX report
Input images: Image 1 is the Web simulation screenshot and is the authoritative geometry/orientation reference. Image 2 is the current candidate F only as a composition reference.
Primary request: redraw the concept aircraft as an exceptionally clear near-vector isometric engineering plate with minimal visual noise, suitable for deterministic HTML/SVG callouts and PDF reduction.
Critical orientation invariant: the nose and +x_b direction are on the LEFT. LEFT is the FRONT tractor propulsor, propeller before the motor. RIGHT is the REAR tail pusher propulsor, propeller after the motor. Do not swap them.
Required architecture: exactly two propellers aligned with the fuselage; one continuous conventional main wing with two halves; one small vertical fin at each outer wing tip; no independent horizontal tailplane; no second wing; slender central fuselage.
Front mechanism: cyan single-axis vertical yaw clevis about z_b, visibly top-and-bottom pivots only. Rear mechanism: orange single-axis transverse pitch clevis about y_b, visibly left-and-right pivots only. Shapes must differ, so meaning survives grayscale printing.
Style/medium: precise aerospace patent drawing with subtle flat fills, clean 1.5 to 2 px dark contour lines, almost no surface texture, white fuselage with light cool-gray wing, black motor and propeller silhouettes, cyan and orange hardware. No glossy CAD reflections, no gradients except extremely subtle form shading, no decorative panel lines.
Composition/framing: landscape 3:2, controlled near-orthographic three-quarter top view with weak perspective, aircraft centered and fills about 74 percent of canvas, both ends fully visible, clean white field and ample callout margins.
Readability constraints: at thumbnail size the single main wing, front tractor, rear pusher, and two differently oriented hinges remain obvious. Use matching three-blade propellers. Keep the two wingtip fins small and clearly attached to the wing tips.
Constraints: no text, no letters, no numbers, no arrows, no axis triad, no watermark, no X-wing, no extra propellers, no tailplane, no cockpit or canopy, no landing gear, no military details, no scene, no heavy shadow, no exploded parts, no fictitious internal mechanism.
```

## 候选 K 提示词

```text
Use case: precise-object-edit
Asset type: traceable candidate K for a graduate engineering LaTeX report
Input images: Image 1 is candidate J and is the edit target. The previously shown Web simulation screenshot remains the engineering orientation reference.
Primary request: preserve candidate J's clean near-vector aircraft topology, but change the viewing angle and mechanism shapes just enough to make the two orthogonal single-axis hinges visually distinct in 3D.
Critical orientation invariant: LEFT remains the FRONT/nose and +x_b direction. LEFT propulsor remains the front tractor with propeller ahead of motor. RIGHT remains the REAR/tail pusher with propeller behind motor. Never swap them.
Change only these items: lower the camera slightly and add modest three-quarter obliquity; make the front cyan mount a tall vertical top-and-bottom clevis about z_b; make the rear orange mount a transverse left-and-right clevis about y_b; keep both wingtip fins small and symmetric; leave more white space around both propellers.
Invariants: keep candidate J's slender fuselage, exactly two matching three-blade propellers, one continuous main wing, no independent horizontal tailplane, near-vector patent/textbook style, white and light-gray materials, thin dark outlines, cyan front hardware, orange rear hardware, no cockpit canopy emphasis.
Constraints: single-axis hinges only, no ring cages, no universal joints, no second rotational ring, no extra fins, no extra wing, no extra rotors, no X-wing, no text, no labels, no arrows, no axis triad, no numbers, no watermark, no scenery, no heavy shadow, no exploded or internal components.
```

## 候选 L 提示词

```text
Use case: precise-object-edit
Asset type: traceable candidate L correcting rotor shaft and propeller handedness for a graduate engineering LaTeX report
Input images: Image 1 is candidate J and is the edit target. Preserve the entire aircraft, camera, composition, wing topology, motors, mounts, lighting, colors, and white background.
Primary request: change only the two propellers and their immediately visible shaft/hub geometry so the counter-rotating rotor-axis convention is mechanically and visually correct.
Fixed aircraft orientation: LEFT is the FRONT/nose and +x_b direction. RIGHT is the REAR/tail.
Front LEFT tractor unit: propeller remains ahead of the motor on the far LEFT. Its rotor angular-momentum/rotation vector is along +x_b, therefore points LEFT in this image. Use a three-blade propeller with blade pitch/handedness consistent with CW when viewed along +x_b.
Rear RIGHT pusher unit: propeller remains behind the motor on the far RIGHT. Its rotor angular-momentum/rotation vector is along -x_b, therefore points RIGHT in this image. Use a three-blade propeller whose blade pitch/handedness is the exact mirror/opposite of the front propeller, consistent with CCW when viewed along +x_b.
Both propeller shaft centerlines must remain coaxial with the fuselage longitudinal axis at zero vector angle. The motors must not tilt, cant, or point off-axis. The front and rear propeller hubs must remain centered on their motor shafts.
Invariants: exactly two propellers; left-front tractor and right-rear pusher; one continuous main wing; two small wingtip fins; cyan front vertical single-axis mount; orange rear transverse single-axis mount; near-vector technical illustration; no redesign of any other component.
Constraints: edit only propeller handedness, hub, and short visible shaft; no arrows, no text, no labels, no numbers, no watermark; no extra blades, no extra motors, no change to camera, no change to airframe or mounts, no motion blur.
```

## 候选 M 生成记录

- 生成工具：`mcp__uTools__utools_zfc4py1y_image_generate`
- 参考图：`docs/03-理论推导/THY-004/fig/01-ai-selected-j.png`
- 输出原始文件：`C:/Users/12631/Downloads/image_1784748715455.png`
- 归档尺寸：`1264 x 848`
- 追溯说明：上游会话只保留了生成工具、参考图和编辑意图，未保留逐字提示词。以下是按会话交接记录恢复的完整语义约束，**不是逐字原始提示词**，因此 M 不作为最终论文证据图。

```text
Use case: precise-object-edit
Asset type: traceable non-zero-deflection candidate for a graduate engineering LaTeX report
Input images: Image 1 is candidate J and is the edit target.
Primary request: preserve the exact airframe, camera, white background, left-front tractor propulsor, right-tail pusher propulsor, single main wing, two wingtip fins, colors, and lighting, but change the two complete motor/propeller assemblies from the zero-angle coaxial pose to visibly different non-zero vectoring poses.
Front motor on the LEFT: rotate the entire cyan-mounted motor, shaft, hub, and propeller as one rigid assembly in the yaw sense associated with the front z_b hinge.
Tail motor on the RIGHT: rotate the entire orange-mounted motor, shaft, hub, and propeller as one rigid assembly in the pitch sense associated with the tail y_b hinge.
Critical invariant: the two motor shaft centerlines must no longer be collinear; show a clearly kinked spatial relation while preserving the front tractor and tail pusher arrangement.
Constraints: exactly two matching three-blade propellers; no labels, no arrows, no axis letters, no extra motors, no universal joints, no airframe redesign, no watermark.
```

## 候选 N 提示词

- 生成工具：`mcp__uTools__utools_zfc4py1y_image_generate`
- 参考图：`configuration-candidate-m-orthogonal-deflection.png`
- 输出原始文件：`C:/Users/12631/Downloads/image_1784748963698.png`
- 归档尺寸：`1264 x 848`

```text
Use case: scientific-educational
Asset type: traceable candidate background for a graduate engineering LaTeX report; deterministic labels and arrows will be overlaid later, so generate no text.
Input images: Image 1 is the edit target and authoritative airframe reference.
Primary request: preserve the exact aircraft, camera, white background, front/left tractor unit, rear/right pusher unit, single main wing, two wingtip fins, colors, lighting, and composition, but correct and exaggerate the two independent single-axis vectoring poses so the two motor shafts are visibly non-collinear and their deflection planes are orthogonal.
Camera/body convention: nose and +x_b point to the LEFT. The viewer sees the aircraft from above and from the +y_b side, so +y_b projects diagonally down toward the viewer and +z_b projects mostly downward.
Front motor on the LEFT: yaw deflection only. Rotate the complete cyan-mounted front motor, nacelle, shaft, and propeller as one rigid assembly by about +18 degrees about the body z_b axis. Its shaft must leave the fuselage centerline sideways within the x_b-y_b plane, toward the near-side wing; do not pitch it up or down.
Tail motor on the RIGHT: pitch deflection only. Rotate the complete orange-mounted tail motor, nacelle, shaft, and propeller as one rigid assembly by about -18 degrees about the body y_b axis. Its shaft must leave the fuselage centerline vertically within the x_b-z_b plane; do not yaw it sideways.
Critical visual invariant: the front and tail shaft centerlines must form an unmistakable spatially kinked, non-collinear pair. The front deflection must read as plan-view yaw; the tail deflection must read as side-view pitch. The two colored hinge axes themselves must remain orthogonal: cyan pivot axis parallel to z_b, orange pivot axis parallel to y_b.
Invariants: exactly two three-blade propellers; left/front is tractor with propeller ahead of its motor; right/tail is pusher with propeller behind its motor; preserve the airframe silhouette and all aerodynamic surfaces; preserve cyan only for front yaw mount and orange only for tail pitch mount.
Style/medium: crisp high-key aerospace CAD textbook illustration on pure white, mechanically plausible, minimal soft shadow.
Constraints: no labels, no arrows, no axis letters, no numbers, no watermark, no extra rings, no universal joints, no extra rotors, no airframe redesign, no swapped front/rear.
Avoid: both shafts collinear, both motors tilted in the same image plane, arbitrary diagonal tilts, pitch motion at the front, yaw motion at the tail, distorted propellers, X-wing ambiguity.
```

## M/N 独立审查结论

- 视觉审查：M 的尾发倾斜更清楚，N 缩印后接近同轴；`M > N`。
- 构型审查：两者均保持左前拉、右尾推、单主翼和两侧翼尖小翼；M 的电机筒与桨刚体一致性略好。
- 物理审查：两者都缺少零位基准、真实三维枢轴线和绑定到正交平面的摆动弧，不能仅凭二维透视证明 `front.rotation.z` 与 `tail.rotation.y`。
- 采用结论：M/N 都不进入论文主图；M 仅作为“AI 能否改善非同轴观感”的最佳留档候选。

## 最终选用的确定性底图

- 源码：`docs/03-理论推导/THY-004/fig/01-web-orthogonal-deflection.html`
- 渲染：直接复用 `simulations/vector-thrust-lab/src/browser/aircraft-view.mjs`
- 固定状态：`delta_f = +18 deg` 绕 `z_b`，`delta_t = +18 deg` 绕 `y_b`
- 输出：`docs/03-理论推导/THY-004/fig/01-web-orthogonal-deflection.png`
- 尺寸：`1600 x 900`
- SHA-256：`9DA7A49AA94F9147A127A3ED0FC6604B91109070A4A590ED702C33253B156ADF`
- 可验证元素：两条零摆角虚线、两条当前电机轴线、前部 `z_b` 枢轴、尾部 `y_b` 枢轴、两个绑定到不同三维平面的摆动弧。

## 独立审查结论

- 视觉审查：`J > K > H > F > G > I`。J 的单主翼、两侧翼尖小翼、缩印和灰度最稳定。
- 架构审查：J 为 A-；K 的前挂架像隔框、尾部多圆点像多连杆，因此不替代 J。
- 物理语义审查：H 的留白最宽，但 J 仍可通过确定性 `z_b/y_b` 轴线、旋转弧和观察方向说明消除歧义。
- 论文采用：Web 确定性底图。J/M 仅作为可追溯 AI 候选；L 证明静态 AI 叶型无法可靠表达旋向，M/N 证明静态透视也无法单独证明正交摆轴，因此 `h_f/h_t`、`z_b/y_b` 与零位基准均由 Web/HTML/TikZ 确定性表达。

## 候选 A 提示词

```text
Use case: scientific-educational
Asset type: reusable concept-aircraft illustration candidate for an engineering LaTeX report
Input image: the provided Web simulation screenshot is the authoritative geometry and orientation reference
Primary request: recreate the same tandem twin-engine fixed-wing concept aircraft as a clean, highly legible three-quarter engineering visualization
Critical orientation invariant: in the reference view, the LEFT propeller/motor assembly is the FRONT tractor engine and the RIGHT propeller/motor assembly is the REAR pusher engine. Preserve this exact front/rear relation. Do not swap them.
Subject: slender central fuselage, one conventional main wing, conventional tail surfaces, two propellers aligned longitudinally with the fuselage, front engine in tractor configuration, rear engine in pusher configuration, orthogonal single-axis vectoring mechanisms around the two motor mounts
Style/medium: premium white-background CAD product rendering, restrained gray aircraft surfaces, cyan vectoring rings and small technical accents
Composition: landscape 3:2, aircraft fills most of frame, three-quarter top view matching the reference, both engines and both propellers clearly separated and readable
Lighting: soft neutral studio lighting, subtle contact shadow only
Constraints: preserve one normal wing and one normal tail set; preserve tandem inline engine layout; mechanically plausible; no extra wings; no duplicated propellers; no X-wing; no labels; no text; no watermark; no scenery; no dramatic perspective distortion
```

## 候选 B 提示词

```text
Use case: scientific-educational
Asset type: reusable technical illustration candidate for an engineering LaTeX report
Input image: the provided Web simulation screenshot is the authoritative geometry and orientation reference
Primary request: transform the exact referenced aircraft into a clear isometric technical illustration emphasizing how the tandem propulsors and vectoring mounts relate to the airframe
Critical orientation invariant: in the reference view, the LEFT motor/propeller is the FRONT tractor unit and the RIGHT motor/propeller is the REAR pusher unit. Keep this exact relationship and do not reverse it.
Style/medium: precise semi-realistic CAD illustration, white to very light gray background, matte graphite fuselage, light gray wings, cyan gimbal arcs, small yellow pivot markers
Composition: landscape 3:2, uncluttered three-quarter view, front and rear propulsors both fully visible, strong silhouette, generous margins for later annotations
Constraints: same single main wing and conventional tail arrangement as reference; exactly two inline propellers; front tractor and rear pusher; no extra aerodynamic surfaces; no X-shaped wing ambiguity; no text; no arrows; no labels; no watermark
```

## 候选 C 提示词

```text
Use case: scientific-educational
Asset type: traceable candidate background for a LaTeX engineering report figure; annotations will be added later in HTML, so generate no text.
Input image: use the currently visible Web-rendered tandem twin vector-thrust aircraft as the geometric reference.
Primary request: create a clean, publication-quality industrial CAD product render of the same concept aircraft, preserving its defining geometry.
Subject: one fixed-wing aircraft with exactly two electric propellers arranged in tandem on the body longitudinal axis. In this camera view, the nose and +x_b direction are to the LEFT. The LEFT motor is the front tractor motor: propeller ahead of its nacelle. The RIGHT motor is the tail pusher motor: propeller behind its nacelle. Keep one main wing and conventional tail surfaces; no extra rotors, no X-wing, no duplicated wings.
Style/medium: high-end aerospace CAD visualization, realistic but schematic, crisp hard-surface modeling, neutral matte graphite and light gray materials with restrained cyan mechanical accents.
Composition/framing: wide 3:2 landscape, aircraft large and centered, three-quarter top view matching the reference orientation, generous clean margin around all propellers and tail surfaces.
Lighting/mood: bright neutral studio lighting, soft contact shadow, white-to-light-gray seamless background, strong silhouette separation.
Constraints: exact two-motor tandem layout; left is FRONT/tractor and right is TAIL/pusher; engineering-plausible proportions; both propellers fully visible; no labels, no arrows, no letters, no numbers, no watermark.
Avoid: futuristic fantasy styling, cockpit canopy emphasis, missiles, landing gear, quadcopter layout, X-shaped wings, extra propellers, reversed front and rear, dark background, dramatic fog, motion blur.
```

## 候选 D 提示词

```text
Use case: scientific-educational
Asset type: second traceable candidate background for a LaTeX engineering figure; precise HTML labels will be overlaid later, so generate no text.
Input image: use the currently visible Web-rendered tandem twin vector-thrust aircraft as the geometry reference.
Primary request: make a very clear aerospace technical illustration that immediately communicates the tandem front-tractor and tail-pusher arrangement.
Subject: one fixed-wing concept aircraft with exactly two electric propulsors on the longitudinal body axis. Camera convention is fixed: nose and +x_b point LEFT. On the LEFT, the front tractor propeller is visibly in front of its motor nacelle. On the RIGHT, the tail pusher propeller is visibly behind its motor nacelle. Show the front gimbal as a cyan ring suggesting rotation about the body vertical z_b axis; show the tail gimbal as an orange ring suggesting rotation about the transverse y_b axis. Do not add arrows or labels.
Style/medium: premium patent-illustration meets modern engineering textbook, clean semi-realistic 3D line-and-wash rendering, sharp silhouette, restrained material shading, white background.
Composition/framing: wide 3:2 landscape, three-quarter top view, aircraft fills about 75 percent of the canvas, all surfaces and both propellers unobstructed, balanced whitespace for later callouts.
Lighting/mood: neutral high-key, soft ambient occlusion only.
Color palette: white, cool gray, graphite; cyan only for front gimbal hardware and orange only for tail gimbal hardware.
Constraints: preserve exact two-motor tandem layout and left-front/right-tail orientation; one main wing and conventional tail surfaces; mechanical gimbals clearly distinct; no text, no letters, no numbers, no watermark.
Avoid: X-wing, delta-wing redesign, quadcopter, extra rotors, coaxial rotors, reversed front and rear, dramatic perspective, dark background, clutter, fantasy aircraft.
```

## 候选 E 提示词

```text
Use case: scientific-educational
Asset type: third traceable candidate background for a LaTeX engineering report figure; no embedded text because labels will be deterministic HTML/SVG.
Input image: use the currently visible Web-rendered aircraft as the configuration reference.
Primary request: create a sophisticated aerospace systems illustration with maximum first-glance readability, combining a realistic aircraft body with subtly exposed vector-thrust mechanisms.
Subject: exactly one fixed-wing tandem twin-engine aircraft. Fixed view convention: nose and +x_b point LEFT. LEFT is the front tractor unit, with propeller ahead of the nacelle. RIGHT is the tail pusher unit, with propeller behind the nacelle. Preserve one main wing and conventional tail surfaces. Make both motor pods and their single-axis gimbal joints visually clear and mechanically plausible, without changing the aircraft architecture.
Style/medium: polished cutaway CAD render for a graduate engineering thesis, realistic hard-surface model with selective translucent fairing around the two motor mounts, crisp edges, subtle technical linework.
Composition/framing: wide 3:2 landscape, near-isometric three-quarter top view, aircraft centered and large, left and right propulsors equally legible, clear empty zones around each end for later callout labels.
Lighting/mood: bright museum-display lighting, white background, soft gray grounding shadow, high local contrast.
Color palette: neutral aluminum and graphite body, cyan highlight only on the front vector mount, amber-orange highlight only on the tail vector mount, restrained green point at the center of mass.
Constraints: exact two propellers only; correct left-front tractor and right-tail pusher order; both propellers fully visible; no arrows, no axis triad, no text, no numbers, no watermark.
Avoid: reversed orientation, extra motors, X-shaped or duplicated wings, science-fiction ornament, dark scene, exploded loose parts, clutter, blur, dramatic lens distortion.
```

## 候选 F 编辑提示词

```text
Use case: precise-object-edit
Asset type: sixth traceable candidate for a LaTeX engineering report
Edit target: the middle of the three currently visible candidates, the clean white technical illustration with a cyan front mount and orange tail mount.
Primary request: change only the two vectoring mounts so they read as two different single-axis gimbals instead of cage-like or universal joints.
Front mount on the LEFT: preserve the front tractor propeller and nacelle, but redesign the cyan mechanism as one simple yaw hinge about the body vertical z_b axis, with an unmistakable top-and-bottom vertical pivot/trunnion arrangement; no second rotational ring.
Tail mount on the RIGHT: preserve the tail pusher propeller and nacelle, but redesign the orange mechanism as one simple pitch hinge about the transverse y_b axis, with an unmistakable left-and-right horizontal pivot/trunnion arrangement; no second rotational ring.
Invariants: keep the exact camera, aircraft silhouette, body, canopy, both halves of the main wing, wingtip fins, lighting, white background, and left-front/right-tail ordering unchanged. Keep exactly two matching three-blade propellers. Left is front tractor; right is tail pusher.
Constraints: cyan only on the front yaw hinge; orange only on the tail pitch hinge; mechanically plausible; crisp engineering illustration; no text, no arrows, no axis labels, no extra rings, no extra rotors, no watermark.
Avoid: universal joints, spherical cages, two-axis gimbals, swapped front and rear, X-wing ambiguity, any redesign of the airframe.
```

## 使用边界

后续若用于论文或正式报告，必须先与 Web 模型逐项复核：前后发位置、拉/推式螺旋桨方向、主翼与尾翼拓扑、两套正交单轴摆转机构以及相机视角。未经复核只能标注为“概念示意”。
