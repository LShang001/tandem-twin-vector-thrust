"""
╔══════════════════════════════════════════════════════════╗
║  纵列双发矢量推力飞行器 · Blender 电影级动画渲染脚本      ║
║  引擎: EEVEE Next (快速高质量) / Cycles 可选              ║
║  用法: blender.exe --background --python anim.py          ║
╚══════════════════════════════════════════════════════════╝
"""
import bpy
import math
import os
import sys

OUT_DIR = os.path.dirname(os.path.abspath(__file__))
RENDER_ENGINE = "BLENDER_EEVEE"  # 改 "CYCLES" 获得照片级（慢很多）

# ═════════════════════════════════════════════════════════
# 0. 清空场景
# ═════════════════════════════════════════════════════════
bpy.ops.wm.read_factory_settings(use_empty=True)


# ═════════════════════════════════════════════════════════
# 1. 飞行器模型 — 精细程序化构建
# ═════════════════════════════════════════════════════════

def add_fuselage():
    """机身：高精度旋成体 + 面板线装饰"""
    # 用贝塞尔曲线旋成
    bpy.ops.curve.primitive_bezier_curve_add()
    profile = bpy.context.active_object
    profile.name = "FuselageProfile"
    profile.data.dimensions = "2D"
    
    # 定义机身截面轮廓（上/下半各半）
    spline = profile.data.splines[0]
    spline.bezier_points[0].co = (0, 0, 0)
    spline.bezier_points[1].co = (0, 0, 0.15)
    
    # 增加控制点：构建流线型旋成体
    spline.bezier_points.add(6)
    pts = spline.bezier_points
    coords = [
        (0,   0,  0.12),   # 机头顶点
        (0.2, 0,  0.22),   # 机头上部
        (0.6, 0,  0.34),   # 最大截面
        (1.2, 0,  0.34),   # 机身中段
        (2.0, 0,  0.28),   # 后段收缩
        (2.6, 0,  0.18),   # 尾锥开始
        (3.2, 0,  0.06),   # 尾锥末端
        (3.4, 0,  0),      # 尾部顶点
    ]
    for i, (x, y, z) in enumerate(coords[:len(pts)]):
        pts[i].co = (x, y, z)
    
    # 加旋成修改器
    bpy.ops.object.modifier_add(type="SCREW")
    profile.modifiers["Screw"].axis = "X"
    profile.modifiers["Screw"].steps = 64
    profile.modifiers["Screw"].render_steps = 96
    profile.modifiers["Screw"].use_normal_calculate = True
    profile.modifiers["Screw"].use_normal_flip = False
    
    bpy.ops.object.modifier_add(type="SUBSURF")
    profile.modifiers["Subdivision"].levels = 1
    profile.modifiers["Subdivision"].render_levels = 2
    
    bpy.ops.object.convert(target="MESH")
    bpy.context.active_object.name = "Fuselage"
    return bpy.context.active_object


def add_canopy():
    """座舱盖：拉伸球体 + 玻璃材质"""
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=0.22, location=(0.55, 0, 0.35),
        segments=32, ring_count=16
    )
    canopy = bpy.context.active_object
    canopy.name = "Canopy"
    canopy.scale = (1.8, 0.8, 0.45)
    return canopy


def add_wings():
    """主翼：用曲线挤出带翼型截面"""
    # 创建翼型截面
    bpy.ops.curve.primitive_bezier_curve_add(location=(0, 0, 0))
    airfoil = bpy.context.active_object
    airfoil.name = "WingProfile"
    airfoil.data.dimensions = "2D"
    
    spline = airfoil.data.splines[0]
    spline.bezier_points.add(6)
    pts = spline.bezier_points
    # 简化的 NACA 翼型轮廓（上弧 12% 厚度，下弧较平）
    foil_pts = [
        (0,     0,  0),       # 前缘
        (0.08,  0,  0.05),    # 上弧前段
        (0.25,  0,  0.065),   # 上弧最大厚度
        (0.5,   0,  0.05),    # 上弧中段
        (0.75,  0,  0.028),   # 上弧后段
        (1.0,   0,  0.002),   # 后缘
        (0.75,  0, -0.015),   # 下弧后段
        (0.25,  0, -0.035),   # 下弧前段
        (0,     0,  0),       # 回到前缘
    ]
    for i, (x, y, z) in enumerate(foil_pts[:len(pts)]):
        pts[i].co = (x, y, z)
    
    # 沿展长方向挤出
    bpy.ops.object.convert(target="MESH")
    bpy.context.active_object.name = "WingCrossSection"
    
    bpy.ops.object.modifier_add(type="SOLIDIFY")
    bpy.context.active_object.modifiers["Solidify"].thickness = 0.004
    bpy.context.active_object.modifiers["Solidify"].offset = 0
    
    # 拉伸成完整机翼
    bpy.ops.object.modifier_add(type="ARRAY")
    bpy.context.active_object.modifiers["Array"].count = 2
    # 使用镜像来创建两侧机翼
    bpy.ops.object.modifier_add(type="MIRROR")
    bpy.context.active_object.modifiers["Mirror"].mirror_object = bpy.data.objects.get("Fuselage")
    
    bpy.ops.object.convert(target="MESH")
    bpy.context.active_object.name = "Wings"
    bpy.context.active_object.scale = (0.001, 1.0, 1.3)  # 粗糙调整
    return bpy.context.active_object


def add_nacelle(location, name):
    """电机舱：圆柱 + 环形进气道装饰"""
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.09, depth=0.32,
        location=location, vertices=32
    )
    nac = bpy.context.active_object
    nac.name = name
    nac.rotation_euler = (math.pi/2, 0, 0)
    
    # 进气道环
    bpy.ops.mesh.primitive_torus_add(
        major_radius=0.09, minor_radius=0.012,
        location=(location[0]+0.17, location[1], location[2]),
        major_segments=32, minor_segments=12
    )
    ring = bpy.context.active_object
    ring.name = f"{name}_IntakeRing"
    ring.rotation_euler = (math.pi/2, 0, 0)
    
    # 喷口环
    bpy.ops.mesh.primitive_torus_add(
        major_radius=0.075, minor_radius=0.01,
        location=(location[0]-0.17, location[1], location[2]),
        major_segments=32, minor_segments=12
    )
    ring2 = bpy.context.active_object
    ring2.name = f"{name}_NozzleRing"
    ring2.rotation_euler = (math.pi/2, 0, 0)
    
    return nac


def add_propeller(location, name, reverse=False):
    """螺旋桨：3 片带扭转的桨叶 + 整流锥 + 桨毂"""
    prop_group = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(prop_group)
    
    hub = bpy.data.objects.new(f"{name}_Hub", None)
    bpy.context.collection.objects.link(hub)
    
    for i in range(3):
        angle = i * 2*math.pi/3
        if reverse:
            angle += math.pi
        
        bpy.ops.mesh.primitive_cube_add(size=0.025)
        blade = bpy.context.active_object
        blade.name = f"{name}_Blade{i}"
        blade.scale = (0.06, 0.38, 0.015)
        blade.location = location
        blade.rotation_euler = (0, angle, 0)
        
        # 模拟扭转：顶点编辑
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.transform.rotate(value=0.35 if not reverse else -0.35,
                                  orient_axis="X",
                                  center_override=location)
        bpy.ops.object.mode_set(mode="OBJECT")
        
        blade.parent = prop_group
    
    # 整流锥
    bpy.ops.mesh.primitive_cone_add(
        radius1=0.04, radius2=0.005, depth=0.12,
        location=(location[0] + (0.06 if reverse else -0.06), location[1], location[2]),
        vertices=24
    )
    cone = bpy.context.active_object
    cone.name = f"{name}_Spinner"
    cone.rotation_euler = (math.pi/2, 0, 0)
    cone.parent = prop_group
    
    # 桨毂
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.035, depth=0.04,
        location=location, vertices=24
    )
    hub_obj = bpy.context.active_object
    hub_obj.name = f"{name}_HubGeo"
    hub_obj.rotation_euler = (math.pi/2, 0, 0)
    hub_obj.parent = prop_group
    
    prop_group.location = location
    return prop_group


def add_landing_gear():
    """起落架：前三点式"""
    gears = []
    for loc, name in [
        ((0.35, 0, -0.38), "NoseGear"),
        ((0.55, 0.35, -0.38), "MainGearR"),
        ((0.55, -0.35, -0.38), "MainGearL"),
    ]:
        # 支柱
        bpy.ops.mesh.primitive_cylinder_add(
            radius=0.012, depth=0.28,
            location=(loc[0], loc[1], loc[2]+0.1),
            vertices=16
        )
        strut = bpy.context.active_object
        strut.name = f"{name}_Strut"
        
        # 机轮
        bpy.ops.mesh.primitive_cylinder_add(
            radius=0.04, depth=0.025,
            location=(loc[0], loc[1], loc[2]-0.02),
            vertices=24
        )
        wheel = bpy.context.active_object
        wheel.name = f"{name}_Wheel"
        wheel.rotation_euler = (0, math.pi/2, 0)
        
        gears.extend([strut, wheel])
    return gears


def add_panel_lines():
    """面板线：在机身上创建细分线条"""
    # 用小环/线来表示主要面板接缝
    lines = []
    for x_pos in [0.3, 0.7, 1.1, 1.6, 2.1, 2.6]:
        bpy.ops.mesh.primitive_torus_add(
            major_radius=0.34, minor_radius=0.003,
            location=(x_pos, 0, 0),
            major_segments=48, minor_segments=4
        )
        ring = bpy.context.active_object
        ring.name = f"PanelLine_{x_pos:.1f}"
        ring.rotation_euler = (0, math.pi/2, 0)
        ring.scale = (1, 1, 0.95 if x_pos < 1.0 else (1.0 if x_pos < 2.0 else 0.5))
        lines.append(ring)
    return lines


# ── 执行建模 ──
print("⏳ 构建飞行器几何体...")
fuselage = add_fuselage()
canopy = add_canopy()
# add_wings 太复杂容易出错，用简化版
bpy.ops.mesh.primitive_cube_add(size=1, location=(0.7, 0, 0))
wing_root = bpy.context.active_object
wing_root.name = "Wing"
wing_root.scale = (0.22, 2.6, 0.03)

nac_f = add_nacelle((1.58, 0, 0), "FrontNacelle")
nac_t = add_nacelle((-1.62, 0, 0), "TailNacelle")
prop_f = add_propeller((1.78, 0, 0), "FrontProp", reverse=False)
prop_t = add_propeller((-1.82, 0, 0), "TailProp", reverse=True)
lg = add_landing_gear()
panel_lines = add_panel_lines()

# 垂尾
bpy.ops.mesh.primitive_cube_add(size=1, location=(0.3, 0, 0.28))
fin = bpy.context.active_object
fin.name = "Fin"
fin.scale = (0.25, 0.015, 0.28)

# 腹鳍
bpy.ops.mesh.primitive_cube_add(size=1, location=(0.3, 0, -0.28))
vfin = bpy.context.active_object
vfin.name = "VentralFin"
vfin.scale = (0.15, 0.012, 0.12)

# 翼尖小翼
for y_sign, s in [(1, "R"), (-1, "L")]:
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0.7, y_sign*2.55, 0.08))
    wl = bpy.context.active_object
    wl.name = f"Winglet{s}"
    wl.scale = (0.08, 0.02, 0.22)
    wl.rotation_euler = (0.15 * y_sign, 0, 0)


# ═════════════════════════════════════════════════════════
# 2. PBR 材质系统
# ═════════════════════════════════════════════════════════

def create_material(name, base_rgba, metallic=0.0, roughness=0.5,
                    emission_rgb=None, emission_strength=0.0,
                    anisotropic=0.0, clearcoat=0.0, subsurface=0.0,
                    specular=0.5):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True  # noqa: BL6-warn
    nodes = mat.node_tree.nodes
    bsdf = nodes["Principled BSDF"]
    
    bsdf.inputs["Base Color"].default_value = base_rgba
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Specular IOR Level"].default_value = specular
    bsdf.inputs["Anisotropic"].default_value = anisotropic
    bsdf.inputs["Coat Weight"].default_value = clearcoat
    bsdf.inputs["Subsurface Weight"].default_value = subsurface
    
    if emission_rgb:
        bsdf.inputs["Emission Color"].default_value = emission_rgb
        bsdf.inputs["Emission Strength"].default_value = emission_strength
    
    return mat


# 材质定义
MATERIALS = {
    "Skin":     create_material("蒙皮", (0.80, 0.82, 0.87, 1.0),
                                metallic=0.82, roughness=0.32,
                                anisotropic=0.35, specular=1.2, clearcoat=0.15),
    "Canopy":   create_material("座舱玻璃", (0.06, 0.10, 0.16, 0.7),
                                metallic=1.0, roughness=0.04, specular=2.0,
                                clearcoat=0.9),
    "Wing":     create_material("机翼蒙皮", (0.68, 0.71, 0.77, 1.0),
                                metallic=0.70, roughness=0.38,
                                anisotropic=0.25, specular=1.0),
    "Nacelle":  create_material("发动机舱", (0.35, 0.38, 0.43, 1.0),
                                metallic=0.92, roughness=0.35,
                                anisotropic=0.5, specular=1.3),
    "Prop":     create_material("碳纤维桨", (0.18, 0.20, 0.24, 1.0),
                                metallic=0.65, roughness=0.45,
                                anisotropic=0.7, specular=0.8),
    "Fin":      create_material("尾翼", (0.71, 0.74, 0.80, 1.0),
                                metallic=0.70, roughness=0.38),
    "Gear":     create_material("起落架", (0.25, 0.27, 0.30, 1.0),
                                metallic=0.95, roughness=0.25),
    "Wheel":    create_material("机轮", (0.08, 0.08, 0.09, 1.0),
                                metallic=0.1, roughness=0.75),
    "Panel":    create_material("面板线", (0.15, 0.17, 0.20, 1.0),
                                metallic=0.3, roughness=0.8),
    "GlowS":    create_material("辉光青", (0.13, 0.82, 0.93, 1.0),
                                metallic=0.0, roughness=0.2,
                                emission_rgb=(0.13, 0.82, 0.93, 1.0),
                                emission_strength=3.5),
    "GlowW":    create_material("辉光暖", (0.98, 0.72, 0.08, 1.0),
                                metallic=0.0, roughness=0.2,
                                emission_rgb=(0.98, 0.72, 0.08, 1.0),
                                emission_strength=3.5),
}

# 材质分配映射
ASSIGN = {
    "Fuselage": "Skin",
    "Canopy": "Canopy",
    "Wing": "Wing",
    "WingletR": "Wing", "WingletL": "Wing",
    "Fin": "Fin", "VentralFin": "Fin",
    "FrontNacelle": "Nacelle", "TailNacelle": "Nacelle",
    "FrontNacelle_IntakeRing": "Nacelle",
    "TailNacelle_IntakeRing": "Nacelle",
    "FrontNacelle_NozzleRing": "Nacelle",
    "TailNacelle_NozzleRing": "Nacelle",
    "*NoseGear*": "Gear", "*MainGear*": "Gear",
    "*Wheel": "Wheel",
    "*Blade*": "Prop",
    "*Spinner": "Prop",
    "PanelLine*": "Panel",
    "*FrontProp*Hub*": "Nacelle",
    "*TailProp*Hub*": "Nacelle",
}

# 执行分配
import fnmatch
for obj_name, mat_name in ASSIGN.items():
    mat = MATERIALS.get(mat_name)
    if mat is None:
        continue
    if "*" in obj_name:
        for o in bpy.data.objects:
            if fnmatch.fnmatch(o.name, obj_name) and o.type == "MESH":
                o.data.materials.clear()
                o.data.materials.append(mat)
    else:
        obj = bpy.data.objects.get(obj_name)
        if obj and obj.type == "MESH":
            obj.data.materials.clear()
            obj.data.materials.append(mat)

# 航行灯小球
nav_lights = []
for loc, color, name in [
    ((0.55, 2.55, 0.08), (0.13, 0.82, 0.93, 1.0), "NavLightR"),
    ((0.55, -2.55, 0.08), (1.0, 0.72, 0.08, 1.0), "NavLightL"),
]:
    bpy.ops.mesh.primitive_uv_sphere_add(radius=0.025, location=loc, segments=16, ring_count=8)
    light = bpy.context.active_object
    light.name = name
    mat_name = "GlowS" if "R" in name else "GlowW"
    light.data.materials.append(MATERIALS[mat_name])
    nav_lights.append(light)


# ═════════════════════════════════════════════════════════
# 3. 环境 — 跑道 + 地面 + 物理天空
# ═════════════════════════════════════════════════════════

print("⏳ 构建环境...")

# ── 跑道 ──
bpy.ops.mesh.primitive_plane_add(size=80, location=(40, 0, -0.02))
runway = bpy.context.active_object
runway.name = "Runway"
runway.scale = (1, 0.15, 1)

mat_runway = create_material("跑道", (0.22, 0.23, 0.25, 1.0),
                             metallic=0.0, roughness=0.85, specular=0.1)
runway.data.materials.append(mat_runway)

# 跑道中线
for i in range(20):
    bpy.ops.mesh.primitive_plane_add(size=3, location=(i*4 + 2, 0, 0.001))
    dash = bpy.context.active_object
    dash.name = f"RWDash_{i}"
    dash.scale = (0.02, 0.006, 1)
    mat_white = create_material(f"白线_{i}", (0.85, 0.86, 0.84, 1.0),
                                roughness=0.9, specular=0.05)
    dash.data.materials.append(mat_white)

# ── 地面 ──
bpy.ops.mesh.primitive_plane_add(size=200, location=(40, 0, -0.05))
ground = bpy.context.active_object
ground.name = "Ground"
mat_ground = create_material("地面", (0.18, 0.24, 0.14, 1.0),
                             metallic=0.0, roughness=0.95, specular=0.02)
ground.data.materials.append(mat_ground)

# ── 世界：Nishita 物理天空 ──
world = bpy.data.worlds.new("PhysicalSky")
bpy.context.scene.world = world
world.use_nodes = True  # noqa: BL6-warn
nodes = world.node_tree.nodes
links = world.node_tree.links
nodes.clear()

sky_tex = nodes.new("ShaderNodeTexSky")
sky_tex.sky_type = "HOSEK_WILKIE"
sky_tex.sun_direction = (0.3, -0.25, 0.9)  # 日落角度
sky_tex.turbidity = 2.8                    # 2-6，低=清朗，高=浑浊/黄昏感
sky_tex.ground_albedo = 0.25

bg = nodes.new("ShaderNodeBackground")
bg.inputs["Strength"].default_value = 1.8

output = nodes.new("ShaderNodeOutputWorld")
links.new(sky_tex.outputs["Color"], bg.inputs["Color"])
links.new(bg.outputs["Background"], output.inputs["Surface"])


# ═════════════════════════════════════════════════════════
# 4. 灯光系统
# ═════════════════════════════════════════════════════════

print("⏳ 布光...")

# 主光 Sun（与天空太阳对齐）
bpy.ops.object.light_add(type="SUN", location=(20, -15, 30))
sun = bpy.context.active_object
sun.name = "KeySun"
sun.data.energy = 3.5
sun.data.angle = math.radians(1.2)  # 柔和的日落/日出阴影

# 发动机体积光效（用聚光灯模拟引擎辉光）
for loc, energy, color, name in [
    ((2.0, 0, 0), 2000, (0.15, 0.55, 0.90), "EngineGlowFront"),
    ((-2.0, 0, 0), 1800, (0.95, 0.50, 0.10), "EngineGlowTail"),
]:
    bpy.ops.object.light_add(type="SPOT", location=loc)
    spot = bpy.context.active_object
    spot.name = name
    spot.data.energy = energy
    spot.data.color = color
    spot.data.spot_size = math.radians(25)
    spot.data.spot_blend = 0.6
    spot.data.use_custom_distance = True
    spot.data.cutoff_distance = 8
    # 体积光需要打开 shadow 和体积
    spot.data.shadow_soft_size = 2.0

# 环境补光
for loc, energy, size, color, name in [
    ((-8, 5, 8), 60, 6, (0.75, 0.80, 1.0), "FillSky"),
    ((5, -8, 2), 35, 4, (0.95, 0.70, 0.45), "FillWarm"),
    ((0, 0, -4), 25, 8, (0.35, 0.45, 0.35), "BounceGround"),
]:
    bpy.ops.object.light_add(type="AREA", location=loc)
    area = bpy.context.active_object
    area.name = name
    area.data.energy = energy
    area.data.size = size
    area.data.color = color


# ═════════════════════════════════════════════════════════
# 5. 体积尾焰（Volumetric Engine Plume）
# ═════════════════════════════════════════════════════════

print("⏳ 构建体积尾焰...")

# 用带体积材质的锥体/圆柱模拟发动机尾焰
for loc_x, name in [(1.95, "FrontPlume"), (-1.95, "TailPlume")]:
    bpy.ops.mesh.primitive_cone_add(
        radius1=0.06, radius2=0.14, depth=0.55,
        vertices=32
    )
    plume = bpy.context.active_object
    plume.name = name
    plume.location = (loc_x, 0, 0)
    plume.rotation_euler = (math.pi/2, 0, 0)
    
    # 体积发射材质
    mat = bpy.data.materials.new(name=f"{name}_Vol")
    mat.use_nodes = True  # noqa: BL6-warn
    n = mat.node_tree.nodes
    l = mat.node_tree.links
    n.clear()
    
    # 用 Emission + 渐变模拟体积
    grad = n.new("ShaderNodeTexGradient")
    grad.gradient_type = "LINEAR"
    
    mapping = n.new("ShaderNodeMapping")
    mapping.inputs["Location"].default_value = (-0.5, 0, 0)
    
    coord = n.new("ShaderNodeTexCoord")
    color_ramp = n.new("ShaderNodeValToRGB")
    cr = color_ramp.color_ramp
    cr.elements[0].color = (0.0, 0.0, 0.0, 1.0)     # 近端透明
    cr.elements[1].color = (0.2, 0.6, 0.95, 1.0)      # 远端亮蓝
    cr.elements.new(0.3).color = (0.05, 0.15, 0.8, 1.0)
    cr.elements.new(0.5).color = (0.1, 0.4, 0.9, 1.0)
    
    emission = n.new("ShaderNodeEmission")
    emission.inputs["Strength"].default_value = 8.0
    
    transparent = n.new("ShaderNodeBsdfTransparent")
    mix = n.new("ShaderNodeMixShader")
    
    output = n.new("ShaderNodeOutputMaterial")
    
    l.new(coord.outputs["Generated"], mapping.inputs["Vector"])
    l.new(mapping.outputs["Vector"], grad.inputs["Vector"])
    l.new(grad.outputs["Fac"], color_ramp.inputs["Fac"])
    l.new(color_ramp.outputs["Color"], emission.inputs["Color"])
    l.new(emission.outputs["Emission"], mix.inputs[2])
    l.new(transparent.outputs["BSDF"], mix.inputs[1])
    l.new(color_ramp.outputs["Alpha"], mix.inputs["Fac"])
    l.new(mix.outputs["Shader"], output.inputs["Surface"])
    
    # 设置混合模式
    mat.blend_method = "BLEND"
    mat.use_backface_culling = False
    
    plume.data.materials.append(mat)


# ═════════════════════════════════════════════════════════
# 6. 粒子系统 — 凝结尾迹 + 翼尖涡
# ═════════════════════════════════════════════════════════

def add_trail_emitter(obj_name, location):
    """在指定位置添加粒子发射器"""
    bpy.ops.mesh.primitive_ico_sphere_add(radius=0.03, location=location, subdivisions=1)
    emitter = bpy.context.active_object
    emitter.name = obj_name
    emitter.hide_render = True
    
    # 粒子系统
    bpy.ops.object.particle_system_add()
    psys = emitter.particle_systems[0]
    psys.name = f"{obj_name}_Particles"
    settings = psys.settings
    settings.type = "EMITTER"
    settings.count = 200
    settings.frame_start = 1
    settings.frame_end = 600
    settings.lifetime = 80
    settings.lifetime_random = 0.4
    settings.emit_from = "VERT"
    settings.normal_factor = 0
    settings.object_align_factor = (-1, 0, 0)  # 向后发射
    settings.particle_size = 0.015
    settings.size_random = 0.5
    settings.use_dynamic_rotation = True
    settings.physics_type = "NEWTON"
    settings.mass = 0.001
    settings.brownian_factor = 3.0
    settings.drag_factor = 0.3
    
    # 渲染为 halos（点光源粒子）
    settings.render_type = "HALO"
    
    return emitter


# 发动机尾迹
trail_f = add_trail_emitter("TrailFront", (2.05, 0, 0))
trail_t = add_trail_emitter("TrailTail", (-2.05, 0, 0))

# 翼尖涡
trail_wr = add_trail_emitter("VortexR", (0.7, 2.55, 0))
trail_wl = add_trail_emitter("VortexL", (0.7, -2.55, 0))


# ═════════════════════════════════════════════════════════
# 7. 相机系统 — 多机位
# ═════════════════════════════════════════════════════════

print("⏳ 设置相机...")

# 主相机
bpy.ops.object.camera_add(location=(7, -5, 3))
main_cam = bpy.context.active_object
main_cam.name = "MainCamera"
main_cam.data.lens = 45
main_cam.data.dof.use_dof = True
main_cam.data.dof.aperture_fstop = 3.5
main_cam.data.dof.focus_distance = 8
main_cam.data.show_limits = True
bpy.context.scene.camera = main_cam

# 追踪飞行器
bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0, 0, 0))
track_target = bpy.context.active_object
track_target.name = "AircraftTrack"

constraint = main_cam.constraints.new(type="TRACK_TO")
constraint.target = track_target
constraint.track_axis = "TRACK_NEGATIVE_Z"
constraint.up_axis = "UP_Y"


# ═════════════════════════════════════════════════════════
# 8. 飞行器整体控制与动画
# ═════════════════════════════════════════════════════════

print("⏳ 装配飞行器层级...")

# 创建根级空对象
bpy.ops.object.empty_add(type="PLAIN_AXES", location=(0, 0, 0))
aircraft_root = bpy.context.active_object
aircraft_root.name = "Aircraft"

# 把所有飞行器部件父子到根节点
aircraft_parts = [
    "Fuselage", "Canopy", "Wing", "WingletR", "WingletL",
    "Fin", "VentralFin",
    "FrontNacelle", "TailNacelle",
    "FrontNacelle_IntakeRing", "TailNacelle_IntakeRing",
    "FrontNacelle_NozzleRing", "TailNacelle_NozzleRing",
    "FrontProp", "TailProp",
    "NoseGear_Strut", "NoseGear_Wheel",
    "MainGearR_Strut", "MainGearR_Wheel",
    "MainGearL_Strut", "MainGearL_Wheel",
    "NavLightR", "NavLightL",
    "FrontPlume", "TailPlume",
    "TrailFront", "TrailTail",
    "VortexR", "VortexL",
]
for name in aircraft_parts:
    obj = bpy.data.objects.get(name)
    if obj:
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = aircraft_root
        bpy.ops.object.parent_set(type="OBJECT", keep_transform=True)

# 面板线也挂上去
for o in bpy.data.objects:
    if "PanelLine" in o.name:
        o.parent = aircraft_root

# 追踪目标跟随飞行器
track_target.parent = aircraft_root


# ═════════════════════════════════════════════════════════
# 9. 飞行动画关键帧
# ═════════════════════════════════════════════════════════

print("⏳ 设置关键帧动画...")

scene = bpy.context.scene
scene.frame_start = 1
scene.frame_end = 720  # 30s @ 24fps
fps = 24

# 初始位置（跑道起点）
aircraft_root.location = (4, 0, 1.5)
aircraft_root.rotation_euler = (0, 0, 0)
aircraft_root.keyframe_insert(data_path="location", frame=1)
aircraft_root.keyframe_insert(data_path="rotation_euler", frame=1)

# ── 动画序列 ──

def kf_loc(rot, frame, loc):
    """设置位置关键帧，自动 ease"""
    aircraft_root.location = loc
    aircraft_root.rotation_euler = rot
    aircraft_root.keyframe_insert(data_path="location", frame=frame)
    aircraft_root.keyframe_insert(data_path="rotation_euler", frame=frame)

# 0-3s（帧 1-72）: 静止展示
kf_loc((0, 0, 0), 1, (4, 0, 1.5))
kf_loc((0, 0, 0), 72, (4, 0, 1.5))

# 3-6s（帧 72-144）: 起飞加速，抬头
kf_loc((math.radians(-10), 0, 0), 100, (12, 0, 2.5))
kf_loc((math.radians(-18), 0, 0), 130, (22, 0, 8))
kf_loc((math.radians(-15), 0, 0), 144, (35, 0, 16))

# 6-10s（帧 144-240）: 爬升
kf_loc((math.radians(-10), 0, 0), 180, (55, 0, 30))
kf_loc((math.radians(-5), 0, 0), 210, (80, 0, 45))
kf_loc((math.radians(-3), 0, 0), 240, (105, 0, 55))

# 10-15s（帧 240-360）: 偏航矢量机动 — 前电机偏转，大半径转弯
kf_loc((math.radians(-2), 0, math.radians(8)), 270, (130, 18, 58))
kf_loc((math.radians(-3), 0, math.radians(22)), 300, (150, 48, 55))
kf_loc((math.radians(-4), 0, math.radians(35)), 330, (162, 82, 52))
kf_loc((math.radians(-2), 0, math.radians(20)), 360, (178, 105, 55))

# 15-20s（帧 360-480）: 滚转 + 俯冲
kf_loc((math.radians(0), 0, math.radians(5)), 390, (200, 110, 62))
kf_loc((math.radians(8), math.radians(30), math.radians(2)), 420, (220, 100, 50))
kf_loc((math.radians(15), math.radians(-25), math.radians(0)), 450, (245, 85, 28))
kf_loc((math.radians(5), 0, math.radians(-3)), 480, (270, 65, 20))

# 20-25s（帧 480-600）: 高速通场
kf_loc((math.radians(-3), 0, math.radians(-2)), 510, (290, 40, 8))
kf_loc((math.radians(-2), 0, 0), 540, (310, 15, 5))
kf_loc((math.radians(-2), 0, math.radians(2)), 570, (330, -10, 4))
kf_loc((math.radians(0), 0, math.radians(5)), 600, (345, -30, 3))

# 25-30s（帧 600-720）: 着陆
kf_loc((math.radians(5), 0, math.radians(8)), 620, (355, -35, 1.8))
kf_loc((math.radians(8), 0, 0), 650, (362, -30, 1.5))
kf_loc((math.radians(5), 0, 0), 680, (370, -25, 1.5))
kf_loc((math.radians(2), 0, 0), 720, (380, -20, 1.5))



# ── 相机动画（多机位切换） ──
main_cam.location = (7, -5, 3)
main_cam.keyframe_insert(data_path="location", frame=1)

cam_positions = [
    (1,   (7, -5, 3)),            # 侧面
    (72,  (10, -3, 5)),           # 起飞追踪
    (144, (15, -8, 8)),           # 爬升远摄
    (240, (18, -12, 10)),         # 偏航机动广角
    (300, (8, -18, 8)),           # 从外侧看转弯
    (360, (-10, -14, 12)),        # 从前方仰视
    (420, (6, 12, 15)),           # 滚转俯视
    (480, (20, -5, 3)),           # 高速通场侧面
    (540, (15, -3, 2.5)),         # 近摄
    (600, (5, -8, 4)),            # 着陆跟拍
    (680, (3, -4, 2)),            # 滑跑近景
]

for frame, loc in cam_positions:
    main_cam.location = loc
    main_cam.keyframe_insert(data_path="location", frame=frame)



# ── 螺旋桨旋转动画 ──
for prop_name, reverse in [("FrontProp", False), ("TailProp", True)]:
    prop = bpy.data.objects.get(prop_name)
    if prop:
        prop.rotation_euler = (0, 0, 0)
        prop.keyframe_insert(data_path="rotation_euler", frame=1)
        total_rot = 360 * 20 * (1 if reverse else -1)  # 20 圈
        prop.rotation_euler = (0, total_rot, 0)
        prop.keyframe_insert(data_path="rotation_euler", frame=720)

# ═════════════════════════════════════════════════════════
# 10. 渲染设置
# ═════════════════════════════════════════════════════════

print("⏳ 配置渲染...")

scene = bpy.context.scene
scene.render.engine = RENDER_ENGINE
scene.render.resolution_x = 1920
scene.render.resolution_y = 1080
scene.render.resolution_percentage = 100
scene.render.fps = fps
scene.render.film_transparent = False

# 色彩管理
scene.view_settings.view_transform = "AgX"
scene.view_settings.look = "AgX - Medium High Contrast"

# 运动模糊
scene.render.use_motion_blur = True
scene.render.motion_blur_shutter = 0.5
scene.render.motion_blur_position = "CENTER"

# EEVEE 默认设置即可

if RENDER_ENGINE == "CYCLES":
    scene.cycles.device = "GPU"
    scene.cycles.samples = 256
    scene.cycles.use_denoising = True
    scene.cycles.denoiser = "OPENIMAGEDENOISE"

# 输出路径
scene.render.filepath = os.path.join(OUT_DIR, "frames", "frame_")
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_depth = "16"
scene.render.image_settings.compression = 15


# ═════════════════════════════════════════════════════════
# 11. 渲染
# ═════════════════════════════════════════════════════════

print("\n" + "=" * 60)
print("  纵列双发矢量推力飞行器 · Blender 电影级动画")
print("=" * 60)
print(f"  引擎:     {RENDER_ENGINE}")
print(f"  分辨率:   {scene.render.resolution_x}×{scene.render.resolution_y}")
print(f"  帧率:     {fps} fps")
print(f"  帧范围:   {scene.frame_start}–{scene.frame_end} ({scene.frame_end/fps:.0f}s)")
print(f"  总帧数:   {scene.frame_end - scene.frame_start + 1}")
print(f"  输出目录: {os.path.join(OUT_DIR, 'frames')}")
print("=" * 60)

# 只渲染少量采样帧来验证
sample_frames = [1, 72, 144, 240, 360, 480, 600, 720]
print(f"\n🎬 先渲染 {len(sample_frames)} 个采样帧验证...\n")

for frame in sample_frames:
    scene.frame_set(frame)
    scene.render.filepath = os.path.join(OUT_DIR, "frames", f"preview_{frame:04d}")
    bpy.ops.render.render(write_still=True)
    print(f"  ✅ 帧 {frame:04d} ({frame/fps:.1f}s)")

print("\n" + "=" * 60)
print("  采样帧渲染完成！")
print(f"  查看: {os.path.join(OUT_DIR, 'frames')}")
print("=" * 60)
print("\n💡 渲染完整动画:")
print("  将脚本顶部 RENDER_ENGINE 改为你想要的引擎")
print("  然后重新运行: blender --background --python anim.py")
print("  (完整 720 帧预计耗时 10-40 分钟，取决于引擎)")
