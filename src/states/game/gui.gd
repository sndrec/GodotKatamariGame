class_name Gui
extends Control


func update_countdown(string: String) -> void:
	if string.is_empty():
		%CountdownDisplay.hide()
		return
	%CountdownDisplay.text = string
	%CountdownDisplay.show()

func prettify_size(inSize: float) -> String:
	var mm = int(floor(inSize * 10)) % 10
	var cm = int(floor(inSize)) % 100
	var m = int(floor(inSize * 0.01)) % 1000
	var km = int(floor(inSize * 0.00001))
	
	if inSize < 1:
		return str(mm) + "mm"
	if inSize < 100:
		return str(cm) + "cm " + str(mm) + "mm"
	if inSize < 100000:
		return str(m) + "m " + str(cm) + "cm " + str(mm) + "mm"
	return str(km) + "km " + str(m) + "m " + str(cm) + "cm " + str(mm) + "mm"
	
func prettify_size_goal(inSize: float) -> String:
	var mm = snapped(inSize * 10, 0.1)
	var cm = snapped(inSize, 0.1)
	var m = snapped(inSize * 0.01, 0.1)
	var km = snapped(inSize * 0.00001, 0.1)
	
	if inSize < 1:
		return str(mm) + "mm"
	if inSize < 100:
		return str(cm) + "cm"
	if inSize < 100000:
		return str(m) + "m"
	return str(km) + "km"

func update_ball_size(diameter_in_cm : float) -> void:
	# "%d / %d" % [gems_collected, total_gems]
	%DiameterDisplay.text = prettify_size(diameter_in_cm)
	

func update_ball_goal(diameter_in_cm : float) -> void:
	# "%d / %d" % [gems_collected, total_gems]
	%GoalDisplay.text = prettify_size_goal(diameter_in_cm)


func update_time_display(ticks: int, time_limit: int) -> void:
	var ticks_remaining = time_limit - ticks
	%TimerHand.rotation = lerpf(PI, -PI, float(ticks_remaining) / float(time_limit))
	var timer_display_val = float(ticks_remaining) / float(Engine.physics_ticks_per_second)
	if timer_display_val > 60:
		timer_display_val = timer_display_val / 60
	
	%TimerDisplay.text = str(ceil(timer_display_val))
