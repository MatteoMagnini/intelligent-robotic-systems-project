-- Put your global variables here

MOVE_STEPS = 5
MAX_VELOCITY = 5
WHEEL_DIST = robot.wheels.axis_length
LIGHT_THRESHOLD = 0.66 -- Not used for this experiment
PROXIMITY_THRESHOLD = 0.0 
n_steps = 0


--[[ This function is executed every time you press the 'execute'
     button ]]
function init()
  
  light_position = {1.5, 0.0, 0.5}
  robot_position = {}
  
	local left_v = robot.random.uniform(0,MAX_VELOCITY)
	local right_v = robot.random.uniform(0,MAX_VELOCITY)
	robot.wheels.set_velocity(left_v,right_v)
  
end

-- Ensure not to exceed MAX_VELOCITY
function limit_v(left_v, right_v)

  function limit(value)
    if (value > MAX_VELOCITY) then
      value = MAX_VELOCITY
    end
    
    if (value < - MAX_VELOCITY) then
      value = - MAX_VELOCITY
    end

    return value
  end
  
  return limit(left_v), limit(right_v)

end

--Random walk
function competence0()
  --By default move randomly
  local left_v = robot.random.uniform(0,MAX_VELOCITY)
  local right_v = robot.random.uniform(0,MAX_VELOCITY)
  
  return left_v, right_v
  
end

--Go towards light
function competence1()
  local subsumption = false
  local light_value = 0
  local light_angle = 0
  local left_v = 0
  local right_v = 0
  
  --Aquire light direction
  for i=1, #robot.light do
    if robot.light[i].value > light_value then
      light_value = robot.light[i].value
      light_angle = robot.light[i].angle
    end
  end
   
  --Go towards light (if present)
  if light_value > 0 then
    subsumption = true
    left_v = MAX_VELOCITY - (light_angle * WHEEL_DIST / 2)
    right_v = MAX_VELOCITY + (light_angle * WHEEL_DIST / 2)
  end
  
  --[[ Random Walk when robot is in a sufficient light spot.
  Just ignore for this experiment.
  
  if light_value > LIGHT_THRESHOLD then
    subsumption = false
  end ]]
  
  return subsumption, limit_v(left_v, right_v)
  
end

--Avoid obstacles
function competence2()
  local subsumption = false
  local proximity_value = 0
  local proximity_angle = 0
  local left_v = 0
  local right_v = 0
  
  --Aquire obstacle direction
  for i=1, #robot.proximity do
    if robot.proximity[i].value > proximity_value then
      proximity_value = robot.proximity[i].value
      proximity_angle = robot.proximity[i].angle
    end
  end
  
  --Reverte the direction.
  if (proximity_angle > 0) then
    proximity_angle = proximity_angle - math.pi / 2
  else 
    proximity_angle = proximity_angle + math.pi / 2
  end
  
  --Go away from obstacle
  if proximity_value > PROXIMITY_THRESHOLD then
    subsumption = true
    left_v = MAX_VELOCITY - (proximity_angle * WHEEL_DIST / 2)
    right_v = MAX_VELOCITY + (proximity_angle * WHEEL_DIST / 2)
  end
  
  left_v, right_v = limit_v(left_v, right_v)
  
  --Adding a little bit of noise to avoide stall
  return subsumption, left_v * robot.random.uniform(0.9,1), right_v * robot.random.uniform(0.9,1)
  
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
	n_steps = n_steps + 1
	if n_steps % MOVE_STEPS == 0 then
  
    --Execute competencies
    left_v0, right_v0 = competence0()
    subsumption1, left_v1, right_v1 = competence1()
    subsumption2, left_v2, right_v2 = competence2()
    
    if (subsumption2) then
      robot.wheels.set_velocity(left_v2,right_v2)
    else
      if (subsumption1) then
        robot.wheels.set_velocity(left_v1,right_v1)
      else
        robot.wheels.set_velocity(left_v0,right_v0)
      end
    end
    
    robot_position = {robot.positioning.position.x, robot.positioning.position.y, robot.positioning.position.z}
    
	end

end


--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	left_v = robot.random.uniform(0,MAX_VELOCITY)
	right_v = robot.random.uniform(0,MAX_VELOCITY)
	robot.wheels.set_velocity(left_v,right_v)
	n_steps = 0
end


--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
  -- calculating the metric.
  euclidean_distance = math.sqrt(math.pow(light_position[1] - robot_position[1], 2) +
    math.pow(light_position[2] - robot_position[2], 2) +
    math.pow(light_position[3] - robot_position[3], 2))
  euclidean_distance = math.floor((euclidean_distance) * 10000) / 10000
    
  print(euclidean_distance .. ", !!marker!!")
end
