#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#include <cstring> // <--- 1. 이 헤더 꼭 추가해주세요! (memcpy용)

extern "C" {
#include "common_data.h"
extern volatile ButtonCommand_t g_btn_cmd_queue;
}

Model::Model() : modelListener(0)
{
}

void Model::tick()
{
    // 데이터가 갱신되었는지 확인
    if (g_robot_data.is_updated)
    {
        g_robot_data.is_updated = false;

        // 2. 수정된 부분: volatile 제거를 위해 memcpy 사용
        RobotData_t currentData;
        // (void*) 형변환을 통해 volatile 성질을 잠시 무시하고 복사합니다.
        memcpy(&currentData, (void*)&g_robot_data, sizeof(RobotData_t));

        if (modelListener != 0)
        {
            modelListener->updateRobotData(currentData);
        }
    }
}

void Model::sendButtonCommand(int cmd)
{
    // 드디어 C 변수에 값을 넣습니다!
    g_btn_cmd_queue = (ButtonCommand_t)cmd;
}
