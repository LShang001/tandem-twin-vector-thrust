"""
Blender 命令行渲染测试
用法: blender.exe --background --python test_render.py
输出: test_output.png
"""
import bpy
import math
import os

# ── 清空默认场景 ──
bpy.ops.wm.read_factory_settings(use_empty=True)

# ═══════════════════════════════════════
# 1. 搭建一个简版飞行器（程序化几何体）
# ═══════════════════════════════════════

# ── 机身（拉长的 UV 球体） ──
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.3, location=(0, 0, 0))
fuselage = bpy.context.active_object
fuselage.name = "Fuselage"
fuselage.scale = (1.5, 0.35, 0.35)

# ── 机翼（拉伸的立方体） ──
bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0))
wing = bpy.context.active_object
wing.name = "Wing"
wing.scale = (0.25, 2.2, 0.04)

# ── 座舱盖 ──
bpy.ops.mesh.primitive_uv_sphere_add(radius=0.18, location=(0.25, 0, 0.22))
canopy = bpy.context.active_object
canopy.name = "Canopy"
canopy.scale = (0.6, 0.55, 0.35)

# ── 前电机舱 ──
bpy.ops.mesh.primitive_cylinder_add(radius=0.1, depth=0.35, location=(1.0, 0, 0))
front_nacelle = bpy.context.active_object
front_nacelle.name = "FrontNacelle"
front_nacelle.rotation_euler = (math.pi/2, 0, 0)

# ── 尾电机舱 ──
bpy.ops.mesh.primitive_cylinder_add(radius=0.1, depth=0.35, location=(-1.0, 0, 0))
tail_nacelle = bpy.context.active_object
tail_nacelle.name = "TailNacelle"
tail_nacelle.rotation_euler = (math.pi/2, 0, 0)

# ── 前螺旋桨（3 片桨叶示意） ──
for i in range(3):
    angle = i * 2 * math.pi / 3
    bpy.ops.mesh.primitive_cube_add(size=0.04, location=(1.18, 0, 0))
    blade = bpy.context.active_object
    blade.name = f"FrontBlade_{i}"
    blade.scale = (0.08, 0.4, 0.02)
    blade.rotation_euler = (0, angle, 0)

# ── 尾螺旋桨（反转方向） ──
for i in range(3):
    angle = i * 2 * math.pi / 3 + math.pi
    bpy.ops.mesh.primitive_cube_add(size=0.04, location=(-1.18, 0, 0))
    blade = bpy.context.active_object
    blade.name = f"TailBlade_{i}"
    blade.scale = (0.08, 0.4, 0.02)
    blade.rotation_euler = (0, angle, 0)

# ── 垂尾 ──
bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0.28))
fin = bpy.context.active_object
fin.name = "Fin"
fin.scale = (0.2, 0.02, 0.25)


# ═══════════════════════════════════════
# 2. PBR 材质
# ═══════════════════════════════════════

def make_material(name, base_color, metallic=0.0, roughness=0.5, emission_color=None, emission_strength=0.0):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True  # noqa: BL6-warn
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = base_color
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if emission_color:
        bsdf.inputs["Emission Color"].default_value = emission_color
        bsdf.inputs["Emission Strength"].default_value = emission_strength
    return mat

# 给各部件赋材质
for obj, (name, rgba, metallic, roughness) in {
    "Fuselage":      ("蒙皮", (0.85, 0.87, 0.91, 1.0), 0.75, 0.35),
    "Wing":          ("机翼", (0.71, 0.74, 0.80, 1.0), 0.65, 0.35),
    "Canopy":        ("座舱", (0.12, 0.18, 0.25, 1.0), 1.0, 0.06),
    "FrontNacelle":  ("电机", (0.39, 0.43, 0.49, 1.0), 0.9, 0.40),
    "TailNacelle":   ("电机", (0.39, 0.43, 0.49, 1.0), 0.9, 0.40),
    "Fin":           ("垂尾", (0.71, 0.74, 0.80, 1.0), 0.65, 0.35),
}.items():
    mat = make_material(name, rgba, metallic, roughness)
    bpy.data.objects[obj].data.materials.append(mat)

# 螺旋桨用深色哑光
prop_mat = make_material("桨叶", (0.30, 0.34, 0.38, 1.0), 0.7, 0.5)
for obj_name in bpy.data.objects:
    if "Blade" in obj_name:
        obj = bpy.data.objects[obj_name]
        obj.data.materials.append(prop_mat)


# ═══════════════════════════════════════
# 3. 灯光
# ═══════════════════════════════════════

# 主光（模拟太阳）
bpy.ops.object.light_add(type="SUN", location=(8, -6, 10))
sun = bpy.context.active_object
sun.data.energy = 5.0
sun.data.angle = math.radians(0.5)  # 锐利阴影

# 补光
bpy.ops.object.light_add(type="AREA", location=(-6, 4, 3))
fill = bpy.context.active_object
fill.data.energy = 80
fill.data.size = 3

# 底部反弹光
bpy.ops.object.light_add(type="AREA", location=(0, 0, -3))
bounce = bpy.context.active_object
bounce.data.energy = 40
bounce.data.size = 5
bounce.data.color = (0.7, 0.8, 1.0)

# 发动机辉光点光
for loc, color in [((1.0, 0, 0), (0.2, 0.6, 0.9)), ((-1.0, 0, 0), (1.0, 0.55, 0.1))]:
    bpy.ops.object.light_add(type="POINT", location=loc)
    l = bpy.context.active_object
    l.data.energy = 120
    l.data.color = color


# ═══════════════════════════════════════
# 4. 相机
# ═══════════════════════════════════════

bpy.ops.object.camera_add(location=(4.5, -4.0, 2.5))
cam = bpy.context.active_object
cam.data.lens = 50  # 标准焦段
cam.data.dof.use_dof = True
cam.data.dof.aperture_fstop = 2.8
cam.data.dof.focus_distance = 5.5

# 让相机对准飞行器
constraint = cam.constraints.new(type="TRACK_TO")
constraint.target = bpy.data.objects["Fuselage"]
constraint.track_axis = "TRACK_NEGATIVE_Z"
constraint.up_axis = "UP_Y"

bpy.context.scene.camera = cam


# ═══════════════════════════════════════
# 5. 世界环境（天空渐变）
# ═══════════════════════════════════════

world = bpy.data.worlds.new("Sky")
bpy.context.scene.world = world
world.use_nodes = True  # noqa: BL6-warn
nodes = world.node_tree.nodes
links = world.node_tree.links

nodes.clear()

# 天空渐变用 Gradient Texture
gradient = nodes.new("ShaderNodeTexGradient")
gradient.gradient_type = "LINEAR"

mapping = nodes.new("ShaderNodeMapping")
mapping.inputs["Location"].default_value = (0, 0, 0.4)
mapping.inputs["Rotation"].default_value = (0, 0, 0)
mapping.inputs["Scale"].default_value = (1, 1, 1)

coord = nodes.new("ShaderNodeTexCoord")
color_ramp = nodes.new("ShaderNodeValToRGB")
color_ramp.color_ramp.elements[0].color = (0.04, 0.06, 0.10, 1.0)   # 天顶深蓝
color_ramp.color_ramp.elements[1].color = (0.35, 0.45, 0.60, 1.0)   # 地平线灰蓝
color_ramp.color_ramp.elements.new(0.5).color = (0.20, 0.28, 0.42, 1.0)

background = nodes.new("ShaderNodeBackground")
background.inputs["Strength"].default_value = 1.5

output = nodes.new("ShaderNodeOutputWorld")

links.new(coord.outputs["Generated"], mapping.inputs["Vector"])
links.new(mapping.outputs["Vector"], gradient.inputs["Vector"])
links.new(gradient.outputs["Fac"], color_ramp.inputs["Fac"])
links.new(color_ramp.outputs["Color"], background.inputs["Color"])
links.new(background.outputs["Background"], output.inputs["Surface"])


# ═══════════════════════════════════════
# 6. 渲染设置
# ═══════════════════════════════════════

scene = bpy.context.scene
scene.render.engine = "CYCLES"
scene.render.resolution_x = 1920
scene.render.resolution_y = 1080
scene.render.resolution_percentage = 100
scene.render.film_transparent = False

# Cycles 设置
scene.cycles.device = "GPU"  # 有 GPU 用 GPU，不支持则自动回退 CPU
scene.cycles.samples = 128   # 测试用低采样，正式可调 1024+
scene.cycles.use_denoising = True
scene.cycles.denoiser = "OPENIMAGEDENOISE"

# 色彩管理
scene.view_settings.view_transform = "AgX"
scene.view_settings.look = "AgX - Medium High Contrast"

# 输出路径：脚本所在目录
out_dir = os.path.dirname(os.path.abspath(__file__))
scene.render.filepath = os.path.join(out_dir, "test_output.png")
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_depth = "16"
scene.render.image_settings.compression = 15

# ── 渲染 ──
print("\n" + "=" * 50)
print("开始渲染...")
print(f"输出: {scene.render.filepath}")
print("=" * 50 + "\n")

bpy.ops.render.render(write_still=True)

print("\n✅ 渲染完成！")
