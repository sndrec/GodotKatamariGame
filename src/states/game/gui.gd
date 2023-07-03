class_name Gui
extends Control

var bulbTextures = [preload("res://assets/tex/ui/katamari_overlay_1.png"), preload("res://assets/tex/ui/katamari_overlay_2.png")]
var bulbs = []

var firstSize := true
var katamariStartSize := 0.0
var katamariSize := 0.0
var katamariGoal := 0.0

func _ready() -> void:	
	var randKatamariBulbs = RandomNumberGenerator.new()
	var bulbCount = randKatamariBulbs.randi_range(4, 9)
	var sizes = []
	for i in range(bulbCount):
		sizes.append(randKatamariBulbs.randf_range(0.25, 1))
	
	sizes.sort()
	
	for i in range(bulbCount):
		var NewBulb = Sprite2D.new()
		NewBulb.texture = bulbTextures[randKatamariBulbs.randi_range(0, bulbTextures.size() - 1)]
		var BulbScale = sizes[bulbCount - i - 1]
		if i == 0:
			BulbScale = 1
			NewBulb.texture = bulbTextures[0]
		var BulbColor = Color.from_hsv(randKatamariBulbs.randf_range(0, 1), 1, 1, 1)
		NewBulb.modulate = BulbColor
		var BulbSpin = randKatamariBulbs.randi_range(0, 1)
		
		var bulbArray = [NewBulb, BulbScale, BulbSpin]
		bulbs.append(bulbArray)
		$Targetsizebg.add_child(NewBulb)

func _process(delta: float) -> void:
	for bulb in bulbs:
		var Sprite = bulb[0] as Sprite2D
		var spin = 0.5
		if bulb[2] == 1:
			spin = -0.5
		
		var progressToGoal = (katamariSize - katamariStartSize) / (katamariGoal - katamariStartSize)
		progressToGoal = remap(progressToGoal, 0, 1, 0.25, 1)
		Sprite.scale = Vector2.ONE * bulb[1] * progressToGoal
		Sprite.rotation += (delta / bulb[1]) * spin

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
	katamariSize = diameter_in_cm
	if firstSize:
		katamariStartSize = diameter_in_cm
		firstSize = false
	%DiameterDisplay.text = prettify_size(diameter_in_cm)
	

func update_ball_goal(diameter_in_cm : float) -> void:
	# "%d / %d" % [gems_collected, total_gems]
	katamariGoal = diameter_in_cm
	%GoalDisplay.text = prettify_size_goal(diameter_in_cm)


func update_time_display(ticks: int, time_limit: int) -> void:
	var ticks_remaining = time_limit - ticks
	%TimerHand.rotation = lerpf(PI, -PI, float(ticks_remaining) / float(time_limit))
	var timer_display_val = float(ticks_remaining) / float(Engine.physics_ticks_per_second)
	if timer_display_val > 60:
		timer_display_val = timer_display_val / 60
	
	%TimerDisplay.text = str(ceil(timer_display_val))
