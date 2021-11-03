//
// Created by Summerpray on 2021/11/2.
//

#include <first_order_filter.h>
#include "M_Gimbal.h"
#include "user_lib.h"
#include <cmath>
extern remote_control RC;

void M_Gimbal::init() {
    Rc = RC.get_remote_control_point();
}






