#pragma once
#include "tw_sha256.h"
#include "tw_link.h"
#include "tw_robot.h"

// Team credentials shared by robot and controller. Copy team_config.example.h to
// team_config.h (git-ignored) and put your own team name, Wi-Fi password and key in it.
#if __has_include("team_config.h")
#include "team_config.h"
#else
#include "team_config.example.h"
#define TW_EXAMPLE_CREDENTIALS 1
#endif
