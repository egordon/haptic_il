from gym.envs.registration import register

register(
    id='FoodManip-v0',
    entry_point='gym_food.envs:FoodEnv',
)
