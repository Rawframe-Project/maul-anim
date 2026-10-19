# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The Godot side of tools/retarget_godot.c: reads its in.txt, builds the
# source and target skeletons and a RetargetModifier3D in its global mode
# whose profile lists the bone names both rigs have, plays each source
# pose and writes each target bone's model rotation to out.txt.
#
# usage: godot --headless --script retarget_godot.gd -- <in.txt> <out.txt>

extends SceneTree

var source: Skeleton3D
var target: Skeleton3D
var poses: Array = []
var lines: PackedStringArray
var at := 0
var pose := -1
var wait := 0
var out: FileAccess


func next_line() -> PackedStringArray:
	var line := lines[at]
	at += 1
	return line.split(" ", false)


func read_rig(count: int) -> Skeleton3D:
	var s := Skeleton3D.new()
	for i in range(count):
		var w := next_line()
		s.add_bone(w[0])
		if int(w[1]) >= 0:
			s.set_bone_parent(i, int(w[1]))
		var q := Quaternion(float(w[5]), float(w[6]), float(w[7]), float(w[8]))
		s.set_bone_rest(i, Transform3D(Basis(q), Vector3(float(w[2]), float(w[3]), float(w[4]))))
	for i in range(count):
		s.reset_bone_pose(i)
	return s


func _init() -> void:
	var args := OS.get_cmdline_user_args()
	lines = FileAccess.get_file_as_string(args[0]).split("\n", false)
	var head := next_line()
	source = read_rig(int(head[1]))
	target = read_rig(int(head[2]))
	var count := int(next_line()[1])
	for k in range(count):
		var rotations: Array = []
		for i in range(source.get_bone_count()):
			var w := next_line()
			rotations.append(Quaternion(float(w[0]), float(w[1]), float(w[2]), float(w[3])))
		poses.append(rotations)
	var profile := SkeletonProfile.new()
	var shared: Array = []
	for i in range(source.get_bone_count()):
		if target.find_bone(source.get_bone_name(i)) >= 0:
			shared.append(source.get_bone_name(i))
	profile.bone_size = shared.size()
	for i in range(shared.size()):
		profile.set_bone_name(i, shared[i])
	var modifier := RetargetModifier3D.new()
	modifier.profile = profile
	modifier.use_global_pose = true
	source.add_child(modifier)
	modifier.add_child(target)
	root.add_child(source)
	out = FileAccess.open(args[1], FileAccess.WRITE)


# Each pose is set, given two frames for the modifier to run, then read.
func _process(_delta: float) -> bool:
	if wait > 0:
		wait -= 1
		return false
	if pose >= 0:
		for i in range(target.get_bone_count()):
			var q := target.get_bone_global_pose(i).basis.get_rotation_quaternion()
			out.store_line("%.9f %.9f %.9f %.9f" % [q.x, q.y, q.z, q.w])
	pose += 1
	if pose == poses.size():
		out.close()
		return true
	for i in range(source.get_bone_count()):
		source.set_bone_pose_rotation(i, poses[pose][i])
	wait = 2
	return false
