#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <touchgfx/Color.hpp>
#include <texts/TextKeysAndLanguages.hpp>

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

void mainscreenView::updateScreen(RobotData_t data)
{
    // 1. 공통 정보 업데이트
    gauge_speed.updateValue(data.speed_kmh, 30);

    Unicode::snprintf(txt_speedBuffer, TXT_SPEED_SIZE, "%d.%d", data.speed_kmh / 100, data.speed_kmh % 100);
    txt_speed.invalidate();

    progress_battery.setValue(data.battery_percent);
    Unicode::snprintf(txt_batteryBuffer, TXT_BATTERY_SIZE, "%d", data.battery_percent);
    txt_battery.invalidate();

    // 2. 텍스트 업데이트
    Unicode::fromUTF8((const uint8_t*)data.caller_name, txt_patientBuffer, TXT_PATIENT_SIZE);
    txt_patient.invalidate();

    Unicode::fromUTF8((const uint8_t*)data.dest_loc, txt_destinationBuffer, TXT_DESTINATION_SIZE);
    txt_destination.invalidate();

    // 3. 상태별 UI 제어
    const char* status_msg = "";
    bool show_btn = false;

    // [수정] T_가 두 번 붙은 ID 사용 (T_ + T_BTN_...)
    TEXTS btn_text_id = T_T_BTN_EMERGENCY;

    switch(data.current_state)
    {
        case STATE_WAITING: // 0
            status_msg = "대기중";
            show_btn = false;
            Unicode::snprintf(txt_patientBuffer, TXT_PATIENT_SIZE, "");
            Unicode::snprintf(txt_destinationBuffer, TXT_DESTINATION_SIZE, "");
            txt_patient.invalidate();
            txt_destination.invalidate();
            break;

        case STATE_HEADING: // 1
            status_msg = "픽업중";
            show_btn = false;
            break;

        case STATE_BOARDING: // 2
            status_msg = "환자 탑승중";
            show_btn = true;
            // [수정] T_T_BTN_BOARDING
            btn_text_id = T_T_BTN_BOARDING;
            break;

        case STATE_RUNNING: // 3
            status_msg = "이동중";
            show_btn = true;
            btn_text_id = T_T_BTN_EMERGENCY;
            break;

        case STATE_STOP: // 4
            status_msg = "비상 정지됨";
            show_btn = true;
            // [수정] T_T_BTN_RESUME
            btn_text_id = T_T_BTN_RESUME;
            break;

        case STATE_ARRIVED: // 5
            status_msg = "목적지 도착";
            show_btn = true;
            btn_text_id = T_T_BTN_BOARDING;
            break;

        case STATE_EXITING: // 6
            status_msg = "안녕히 가세요";
            show_btn = false;
            break;

        case STATE_CHARGING: // 7
            status_msg = "충전중";
            show_btn = false;
            break;

        default:
            status_msg = "상태 불명";
            show_btn = false;
            break;
    }

    Unicode::fromUTF8((const uint8_t*)status_msg, txt_StatusBuffer, TXT_STATUS_SIZE);
    txt_Status.invalidate();

    btn.setLabelText(touchgfx::TypedText(btn_text_id));
    btn.invalidate();

    btn.setVisible(show_btn);
    btn.invalidate();
}

void mainscreenView::push_btn()
{
    presenter->userClickedStart();
}
