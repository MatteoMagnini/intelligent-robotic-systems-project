-- Put your global variables here.

MOVE_STEPS = 5
MAX_VELOCITY = 5
FILENAME = "../data/Qtable-obstacle-avoidance.csv"
WHEEL_DIST = robot.wheels.axis_length
n_steps = 0


--- This function is executed every time you press the 'execute' button.
function init()

  Qlearning = require "Qlearning"
  
  total_state_acquisition = 0
  obstacle_free_acquisition = 0
  
  threshold = 0.05
  
	local left_v = robot.random.uniform(0,MAX_VELOCITY)
  local right_v = robot.random.uniform(0,MAX_VELOCITY)
  
  -- States: each direction can be 0 or 1, a state is a combination of those.
  -- So in total the states are 2^8 = 256.
  sensor_direction_names = {"NNW", "WNW", "WSW", "SSW", "SSE", "ESE", "ENE", "NNE"}
  number_of_states = math.pow(2, #sensor_direction_names)
  sectors = {
    {1,2,3},
    {4,5,5},
    {7,8,9},
    {10,11,12},
    {13,14,15},
    {16,17,18},
    {19,20,21},
    {22,23,24},
  }
  
  -- Actions: 5 in total
  -- Threre is no symmetry: a vector direction cannot be cancelled by a opposite vector.
  -- In this way we avoid stupid behaviours such that go forward and then immidiatly backward.
  velocity_direction_names = {"WNW", "NNW", "N", "NNE", "ENE"}
  velocity_directions = {
    ["WNW"] = math.pi / 4, -- 45 degree
    ["NNW"] = math.pi / 8, -- 22.5 degree
    ["N"] = 0,
    ["NNE"] = - math.pi / 8, -- -22.5 degree
    ["ENE"] = - math.pi / 4, -- 45 degree
  }
  
  
  number_of_actions = #velocity_direction_names
  
  Q_table = {}
  
  -- Dimension: 256 x 5 = 1280 values.
  Q_table = Qlearning.load_Q_table(FILENAME)
  
	robot.wheels.set_velocity(left_v,right_v)
  
end

-- Random walk
function competence0()
  --By default move randomly
  local left_v = robot.random.uniform(0,MAX_VELOCITY)
  local right_v = robot.random.uniform(0,MAX_VELOCITY)
  
  return left_v, right_v
  
end

-- Obstacle avoidance
function competence1()

  ---------------------
  -- Inner functions --
  ---------------------
  function action_to_velocity(action)

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
    
    local angle = velocity_directions[velocity_direction_names[action]]
    local left_v = MAX_VELOCITY - (angle * WHEEL_DIST / 2)
    local right_v = MAX_VELOCITY + (angle * WHEEL_DIST / 2)
    
    left_v, right_v = limit_v(left_v,right_v)
    
    return left_v, right_v
    
  end
  
  function get_state()
  
    -- State goes from 1 to 256.
    -- 1 equals that all sensors are 1.
    local new_state = 1
    
    for i = 1, #sectors do
      for j = 1, #sectors[i] do
        if robot.proximity[sectors[i][j]].value > threshold then
          new_state = new_state + math.pow(2,i-1)
          break;
        end
      end
    end
    
    return new_state
    
  end
  
  ----------------------------
  -- End of inner functions --
  ----------------------------

  local state = get_state()
  local action = Qlearning.get_best_action(state, Q_table)
  local subsumption = true
  
  total_state_acquisition = total_state_acquisition + 1
  
  if state == 1 then
    subsumption = false 
  else
    obstacle_free_acquisition = obstacle_free_acquisition + 1
  end
  
  local left_v, right_v = action_to_velocity(action)
  
  return subsumption, left_v * robot.random.uniform(0.9,1), right_v * robot.random.uniform(0.9,1)

end


--- This function is executed at each time step.
-- It must contain the logic of your controller.
function step()
	n_steps = n_steps + 1
  
  -- Perform action
  if n_steps % MOVE_STEPS == 0 then
  
    left_v0, right_v0 = competence0()
    subsumption1, left_v1, right_v1 = competence1()
    
    if (subsumption1) then
      robot.wheels.set_velocity(left_v1,right_v1)
    else
      robot.wheels.set_velocity(left_v0,right_v0)
    end

	end
 
end


--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	local left_v = 0
	local right_v = 0
	robot.wheels.set_velocity(left_v,right_v)
	n_steps = 0
end


--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
   metric = math.floor((obstacle_free_acquisition/total_state_acquisition) * 10000) / 10000
   print(metric .. ", !!marker!!")
   
end
