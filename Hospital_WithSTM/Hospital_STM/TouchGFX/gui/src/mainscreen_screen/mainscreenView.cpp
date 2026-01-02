#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <touchgfx/Color.hpp>
#include <texts/TextKeysAndLanguages.hpp>

// 생성자 이름 수정
mainscreenView::mainscreenView()
{
}

void mainscreenView::setupScreen()
{
    mainscreenViewBase::setupScreen();
}

void mainscreenView::tearDownScreen()
{
    mainscreenViewBase::tearDownScreen();
}

// 함수 이름 수정
void mainscreenView::updateScreen(RobotData_t data)
{
    // 1. 공통 정보 업데이트 (속도, 배터리)
    gauge_speed.setValue(data.speed_kmh);
    Unicode::snprintf(txt_speedBuffer, TXT_SPEED_SIZE, "%d", data.speed_kmh);
    txt_speed.invalidate();

    progress_battery.setValue(data.battery_percent);
    Unicode::snprintf(txt_batteryBuffer, TXT_BATTERY_SIZE, "%d", data.battery_percent);
    txt_battery.invalidate();

    // ----------------------------------------------------------------
    // [수정] if(data.is_updated) 삭제! (무조건 실행되도록 변경)
    // ----------------------------------------------------------------

    // A. 호출자 & 목적지 정보 표시
    // "테스트" 글자를 덮어쓰게 됩니다.
    Unicode::fromUTF8((const uint8_t*)data.caller_name, txt_patientBuffer, TXT_PATIENT_SIZE);
    txt_patient.invalidate();

    Unicode::fromUTF8((const uint8_t*)data.dest_loc, txt_destinationBuffer, TXT_DESTINATION_SIZE);
    txt_destination.invalidate();

    // B. 상태별 텍스트 및 버튼 제어
    const char* status_msg = "";
    bool show_btn = false;

    switch(data.current_state)
    {
        case STATE_WAITING: // 0
            status_msg = "대기중";
            show_btn = false;
            // 대기 중엔 정보 지우기
            Unicode::snprintf(txt_patientBuffer, TXT_PATIENT_SIZE, "");
            Unicode::snprintf(txt_destinationBuffer, TXT_DESTINATION_SIZE, "");
            txt_patient.invalidate();      // <--- invalidate 꼭 필요
            txt_destination.invalidate();  // <--- invalidate 꼭 필요
            break;

        case STATE_HEADING: // 1
            status_msg = "픽업중";
            show_btn = false;
            break;

        case STATE_BOARDING: // 2
            status_msg = "환자 탑승중";
            show_btn = true;
            break;

        case STATE_RUNNING: // 3
            status_msg = "이동중";
            show_btn = true;
            break;

        case STATE_STOP: // 4
            status_msg = "비상 정지됨";
            show_btn = true;
            break;

        case STATE_ARRIVED: // 5
            status_msg = "목적지 도착";
            show_btn = true;
            break;

        case STATE_EXITING: // 6
            status_msg = "안녕히 가세요";
            show_btn = false;
            break;

        case STATE_CHARGING: // 7
            status_msg = "충전중";
            show_btn = false;
            break;
    }

    // 상태 텍스트 적용
    Unicode::fromUTF8((const uint8_t*)status_msg, txt_StatusBuffer, TXT_STATUS_SIZE);
    txt_Status.invalidate();

    // 버튼 숨김/표시 적용
    btn.setVisible(show_btn);
    btn.invalidate();
}

void mainscreenView::sendStartCommand()
{
    // Presenter에게 "버튼 눌렸어!"라고 알림
    presenter->userClickedStart();
}
