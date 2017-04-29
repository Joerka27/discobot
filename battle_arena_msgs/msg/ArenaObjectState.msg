uint8 PLAYER = 0
uint8 ROCKET = 1
uint8 BANANA = 2
uint8 SENTRY = 3
#uint8 TOKEN = 4
uint8 BOX = 5
uint8 WALL = 6
uint8 BASE = 7
uint8 MINE = 8

uint8 TOKEN_SHIELD = 10
uint8 TOKEN_HP = 11
uint8 TOKEN_ROCKET = 12
uint8 TOKEN_BANANA = 13
uint8 TOKEN_MINE = 14
uint8 TOKEN_COLLECTIBLE_TREASURE = 15

uint8 ANIMATION_EXPLOSION = 50
uint8 ANIMATION_SHIELD_UPGRADE = 51
uint8 ANIMATION_HP_UPGRADE = 52
uint8 ANIMATION_BANANA = 53
uint8 ANIMATION_BOX_COLLECTED = 54
uint8 ANIMATION_PLAYER_DIED = 55


string name
int32 object_id
int32 team_id
uint8 type


battle_arena_msgs/Pose pose

float32 x_dimension
float32 y_dimension


# parameters for the different types

string player_name

float32 sentry_vision_radius # if you get closer than this, the sentry gun will see you

float32 shield_radius      # if you get closer than this, you will get the shield upgrade
float32 shield_upgrade       # amount of shield given by this icon
float32 shield_remaining_recharge_time  # time until shield is ready again

float32 box_score            # number of points you get for fetching this box

float32 player_shield # also sent in PlayerState, hack for video
float32 player_hp
