#include "Irrigation4Channel.h"
#include "Mlt16IoCommunication.h"

Irrigation4Channel::Irrigation4Channel(MltFlowSensor *_myFlowBoard, IrrigationParams *_myParams, IrrigationParams *_preParams)
{
    myFlowBoard = _myFlowBoard;
    myParams = _myParams;
    preParams = _preParams;
    Serial.println("Irrigation4Channel initialized");
}

//====================================================================
// Main Loop
//====================================================================
void Irrigation4Channel::loop()
{
    // Đầu tiên, ta cập nhật giá trị đã lọc của tất cả input
    myFlowBoard->updateFilteredInputs();
    updateCommunicationValue();
    updateInputCommunicationValue();

    // Điều khiển cấp nước cho bồn nước
    autoWaterSupplyControl();

    // Điều khiển cấp nước cho bồn pha (mixing) cho Tank1 và Tank2
    autoMixingControl_Tank1();
    autoMixingControl_Tank2();

    // Điều khiển tưới cho Tank1 và Tank2
    autoIrrigationControl_Tank1();
    autoIrrigationControl_Tank2();

    // Lấy giá trị EC từ sensor cho từng bồn pha
    float currentEC1 = Mlt16IoCommunication.getCurrentEc1();
    float currentEC2 = Mlt16IoCommunication.getCurrentEc2();

    // Điều khiển pha EC dosing tự động với bảo vệ cho Tank1 và Tank2
    autoECControl_Tank1(currentEC1, (float)myParams->iri.setECTank1,
                        myParams->iri.dosingTimeTank1, myParams->iri.waitTimeTank1,
                        myParams->iri.ratioTank1A, myParams->iri.ratioTank1B);
    autoECControl_Tank2(currentEC2, (float)myParams->iri.setECTank2,
                        myParams->iri.dosingTimeTank2, myParams->iri.waitTimeTank2,
                        myParams->iri.ratioTank2A, myParams->iri.ratioTank2B);
}

//====================================================================
// Communication Update Functions
//====================================================================
void Irrigation4Channel::updateCommunicationValue()
{
    myParams->iri.dosingTimeTank1 = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK1_DOSING_TIME);
    myParams->iri.waitTimeTank1 = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK1_WAIT_TIME);
    myParams->iri.ratioTank1A = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK1_RATIO_A);
    myParams->iri.ratioTank1B = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK1_RATIO_B);
    myParams->iri.setECTank1 = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK1_SET_EC);

    myParams->iri.dosingTimeTank2 = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK2_DOSING_TIME);
    myParams->iri.waitTimeTank2 = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK2_WAIT_TIME);
    myParams->iri.ratioTank2A = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK2_RATIO_A);
    myParams->iri.ratioTank2B = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK2_RATIO_B);
    myParams->iri.setECTank2 = Mlt16IoCommunication.mbTcp->holdingRegisterRead(HOLDING_TANK2_SET_EC);

    // DB_PRINTLN("Holding registers updated.");
}

void Irrigation4Channel::updateInputCommunicationValue()
{
    for (int i = 0; i < 9; i++)
    {
        bool sensorStatus = myFlowBoard->getFilteredInputValue(i);
        Mlt16IoCommunication.mbTcp->coilWrite(COIL_INPUT_STATUS(i), sensorStatus);
    }
    Mlt16IoCommunication.mbTcp->inputRegisterWrite(INPUT_SENSOR1_EC, (int)(Mlt16IoCommunication.getCurrentEc1()));
    Mlt16IoCommunication.mbTcp->inputRegisterWrite(INPUT_SENSOR2_EC, (int)(Mlt16IoCommunication.getCurrentEc2()));
    Mlt16IoCommunication.mbTcp->inputRegisterWrite(INPUT_SENSOR1_PH, (int)(Mlt16IoCommunication.getCurrentPh1()));
    Mlt16IoCommunication.mbTcp->inputRegisterWrite(INPUT_SENSOR2_PH, (int)(Mlt16IoCommunication.getCurrentPh2()));
    Mlt16IoCommunication.mbTcp->inputRegisterWrite(INPUT_SENSOR1_TEMP, (int)(Mlt16IoCommunication.getCurrentTemp1()));
    Mlt16IoCommunication.mbTcp->inputRegisterWrite(INPUT_SENSOR2_TEMP, (int)(Mlt16IoCommunication.getCurrentTemp2()));

    // DB_PRINTLN("Input registers updated with sensor values.");
}

//====================================================================
// Device Control Functions (Output + Modbus Status)
//====================================================================

void Irrigation4Channel::setWaterSupplyPump(bool state)
{
    // Kiểm tra nếu trạng thái hiện tại đã bằng với trạng thái cần set thì bỏ qua
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_WATER_IN) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_WATER_IN, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_WATER_IN, state);
    DB_PRINTLN(state ? "Water Supply Pump ON" : "Water Supply Pump OFF");
}

bool Irrigation4Channel::getWaterSupplyPump()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_WATER_IN);
}

void Irrigation4Channel::setMixingPump(bool state)
{
    bool valve1State = getMixingValve1();
    bool valve2State = getMixingValve2();
    bool currentPumpState = myFlowBoard->getRelayStatus(COIL_OUTPUT_MIXING_PUMP);

    // Mixing pump chỉ được bật nếu state là true VÀ ít nhất một van chia đang mở
    if (state && (valve1State || valve2State) && !currentPumpState)
    {
        myFlowBoard->setRelay(COIL_OUTPUT_MIXING_PUMP, true);
        Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_MIXING_PUMP, true);
        DB_PRINTLN("Mixing Pump ON (at least one valve open)");
    }
    // Mixing pump bị tắt nếu không có van chia nào mở hoặc state là false
    else if ((!valve1State && !valve2State) || !state)
    {
        if (currentPumpState)
        {
            myFlowBoard->setRelay(COIL_OUTPUT_MIXING_PUMP, false);
            Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_MIXING_PUMP, false);
            DB_PRINTLN("Mixing Pump OFF (no valves open or forced off)");
        }
    }
}

bool Irrigation4Channel::getMixingPump()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_MIXING_PUMP);
}

// void Irrigation4Channel::setMixingPump1(bool state)
// {
//     if (myFlowBoard->getRelayStatus(COIL_OUTPUT_MIXING_PUMP) == state)
//         return;
//     myFlowBoard->setRelay(COIL_OUTPUT_MIXING_PUMP, state);
//     Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_MIXING_PUMP, state);
//     DB_PRINTLN(state ? "Mixing Pump 1 ON" : "Mixing Pump 1 OFF");
// }

// bool Irrigation4Channel::getMixingPump1()
// {
//     return myFlowBoard->getRelayStatus(COIL_OUTPUT_MIXING_PUMP);
// }

void Irrigation4Channel::setMixingValve1(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_VAN_CHIA1) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_VAN_CHIA1, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_VAN_CHIA1, state);
    DB_PRINTLN(state ? "Mixing Valve 1 ON" : "Mixing Valve 1 OFF");
}

bool Irrigation4Channel::getMixingValve1()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_VAN_CHIA1);
}

void Irrigation4Channel::setIrrigationPump1(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_IRRIGATION_PUMP1) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_IRRIGATION_PUMP1, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_IRRIGATION_PUMP1, state);
    DB_PRINTLN(state ? "Irrigation Pump 1 ON" : "Irrigation Pump 1 OFF");
}

bool Irrigation4Channel::getIrrigationPump1()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_IRRIGATION_PUMP1);
}

void Irrigation4Channel::setIrrigationValve1(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_IRRIGATION_VALVE1) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_IRRIGATION_VALVE1, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_IRRIGATION_VALVE1, state);
    DB_PRINTLN(state ? "Irrigation Valve 1 ON" : "Irrigation Valve 1 OFF");
}

bool Irrigation4Channel::getIrrigationValve1()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_IRRIGATION_VALVE1);
}

// void Irrigation4Channel::setMixingPump2(bool state)
// {
//     if (myFlowBoard->getRelayStatus(COIL_OUTPUT_MIXING_PUMP) == state)
//         return;
//     myFlowBoard->setRelay(COIL_OUTPUT_MIXING_PUMP, state);
//     Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_MIXING_PUMP, state);
//     DB_PRINTLN(state ? "Mixing Pump 2 ON" : "Mixing Pump 2 OFF");
// }

// bool Irrigation4Channel::getMixingPump2()
// {
//     return myFlowBoard->getRelayStatus(COIL_OUTPUT_MIXING_PUMP);
// }

void Irrigation4Channel::setMixingValve2(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_VAN_CHIA2) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_VAN_CHIA2, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_VAN_CHIA2, state);
    DB_PRINTLN(state ? "Mixing Valve 2 ON" : "Mixing Valve 2 OFF");
}

bool Irrigation4Channel::getMixingValve2()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_VAN_CHIA2);
}

void Irrigation4Channel::setIrrigationPump2(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_IRRIGATION_PUMP2) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_IRRIGATION_PUMP2, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_IRRIGATION_PUMP2, state);
    DB_PRINTLN(state ? "Irrigation Pump 2 ON" : "Irrigation Pump 2 OFF");
}

bool Irrigation4Channel::getIrrigationPump2()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_IRRIGATION_PUMP2);
}

void Irrigation4Channel::setIrrigationValve2(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_IRRIGATION_VALVE2) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_IRRIGATION_VALVE2, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_IRRIGATION_VALVE2, state);
    DB_PRINTLN(state ? "Irrigation Valve 2 ON" : "Irrigation Valve 2 OFF");
}

bool Irrigation4Channel::getIrrigationValve2()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_IRRIGATION_VALVE2);
}

void Irrigation4Channel::setFertilizerPump1A(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_FERT_PUMP1A) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_FERT_PUMP1A, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_FERT_PUMP1A, state);
    DB_PRINTLN(state ? "Fertilizer Pump 1A ON" : "Fertilizer Pump 1A OFF");
}

bool Irrigation4Channel::getFertilizerPump1A()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_FERT_PUMP1A);
}

void Irrigation4Channel::setFertilizerPump1B(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_FERT_PUMP1B) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_FERT_PUMP1B, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_FERT_PUMP1B, state);
    DB_PRINTLN(state ? "Fertilizer Pump 1B ON" : "Fertilizer Pump 1B OFF");
}

bool Irrigation4Channel::getFertilizerPump1B()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_FERT_PUMP1B);
}

void Irrigation4Channel::setFertilizerPump2A(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_FERT_PUMP2A) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_FERT_PUMP2A, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_FERT_PUMP2A, state);
    DB_PRINTLN(state ? "Fertilizer Pump 2A ON" : "Fertilizer Pump 2A OFF");
}

bool Irrigation4Channel::getFertilizerPump2A()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_FERT_PUMP2A);
}

void Irrigation4Channel::setFertilizerPump2B(bool state)
{
    if (myFlowBoard->getRelayStatus(COIL_OUTPUT_FERT_PUMP2B) == state)
        return;
    myFlowBoard->setRelay(COIL_OUTPUT_FERT_PUMP2B, state);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_OUTPUT_FERT_PUMP2B, state);
    DB_PRINTLN(state ? "Fertilizer Pump 2B ON" : "Fertilizer Pump 2B OFF");
}

bool Irrigation4Channel::getFertilizerPump2B()
{
    return myFlowBoard->getRelayStatus(COIL_OUTPUT_FERT_PUMP2B);
}

//---------------------------------------------------------------------
// Auto Control Functions: Water Supply for Bồn Nước, Mixing, Irrigation
//---------------------------------------------------------------------

// Auto Water Supply for Bồn Nước:
// Logic: Bơm nước sẽ tắt chỉ khi cả 2 cảm biến (INPUT_WATER_TANK_LOW và INPUT_WATER_TANK_HIGH) đều true (nghĩa là bồn đầy).
// Nếu nước nằm ở mức giữa (low true, high false) thì mặc định tắt, nhưng nếu user tắt rồi bật lại chế độ auto (edge-trigger),
// thì sẽ kích hoạt chế độ bơm đầy nước.
void Irrigation4Channel::autoWaterSupplyControl()
{
    bool waterLow = myFlowBoard->getFilteredInputValue(INPUT_WATER_TANK_LOW);
    bool waterHigh = myFlowBoard->getFilteredInputValue(INPUT_WATER_TANK_HIGH);

    // Nếu bồn nước đầy, ép tắt bơm ngay lập tức
    if (waterHigh)
    {
        setWaterSupplyPump(false);
        DB_PRINTLN("Water Tank FULL: Pump forced OFF (always active).");
        return; // Ngăn các logic auto khác chạy
    }

    // Các logic tự động còn lại chỉ được thực thi khi bồn chưa đầy
    bool autoWaterEnabled = Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_WATER_SUPPLY);
    static bool prevAutoWaterEnabled = false;
    static bool forceFill = false;
    // prevState: 0 = Pump OFF, 1 = Pump ON, 2 = forceFill triggered; -1 = uninitialized.
    static int prevState = -1;
    int currentState = prevState;

    // Edge-trigger: khi auto vừa bật và bồn chưa đầy nhưng có nước ở vị trí low, kích hoạt forceFill.
    if (!prevAutoWaterEnabled && autoWaterEnabled && waterLow)
    {
        forceFill = true;
        if (prevState != 2)
        {
            DB_PRINTLN("Auto Water Supply forceFill triggered.");
            prevState = 2;
        }
    }
    prevAutoWaterEnabled = autoWaterEnabled;

    if (autoWaterEnabled)
    {
        // Vì trường hợp waterHigh đã được xử lý ở trên nên phần này chỉ xử lý khi bồn chưa đầy
        if (forceFill || !waterLow)
        {
            setWaterSupplyPump(true);
            currentState = 1;
            if (currentState != prevState)
            {
                DB_PRINTLN("ForceFill or water not detected: Water Supply Pump ON (auto mode).");
            }
        }
        else
        {
            setWaterSupplyPump(false);
            currentState = 0;
            if (currentState != prevState)
            {
                DB_PRINTLN("Water Tank NORMAL (between): Water Supply Pump OFF (auto mode).");
            }
        }
        prevState = currentState;
    }
    else
    {
        // Khi auto mode tắt, cập nhật giá trị theo chế độ manual.
        bool manualValue = Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_WATER_IN);
        if (getWaterSupplyPump() != manualValue)
            setWaterSupplyPump(manualValue);
        prevState = -1;
    }
}

// Auto Mixing (Cấp Nước cho Bồn Pha) cho Tank1:
// Logic: Bơm cấp nước (mixing pump + van) chỉ tắt khi cả 2 cảm biến của bồn pha đều báo true (bồn pha đầy).
// Nếu mức nước nằm giữa, mặc định tắt, trừ khi có edge-trigger để fill.
void Irrigation4Channel::autoMixingControl_Tank1()
{
    bool mixingLow1 = myFlowBoard->getFilteredInputValue(INPUT_MIXING_TANK1_LOW);
    bool mixingHigh1 = myFlowBoard->getFilteredInputValue(INPUT_MIXING_TANK1_HIGH);
    bool mixingHigh2 = myFlowBoard->getFilteredInputValue(INPUT_MIXING_TANK2_HIGH); // Kiểm tra bồn 2

    // Nếu cả hai bồn đều đầy, không cho phép mở van chia hoặc bơm
    if (mixingHigh1 && mixingHigh2)
    {
        setMixingValve1(false);
        setMixingPump(false); // Tắt bơm vì không có van nào cần mở
        DB_PRINTLN("Both Tanks FULL: Mixing Pump and Valve 1 forced OFF.");
        return;
    }

    // Nếu chỉ bồn 1 đầy, tắt van chia 1 và để bồn 2 quyết định bơm
    if (mixingHigh1)
    {
        setMixingValve1(false);
        setMixingPump(false); // Tắt tạm thời, bồn 2 sẽ kiểm tra lại
        DB_PRINTLN("Mixing Tank 1 FULL: Valve 1 forced OFF.");
        return;
    }

    bool autoMixing1 = Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_MIXING_TANK1);
    static bool prevAutoMixing1 = false;
    static bool forceFillMix1 = false;
    static int prevState = -1;
    int currentState = prevState;

    if (!prevAutoMixing1 && autoMixing1 && mixingLow1 && !mixingHigh1)
    {
        forceFillMix1 = true;
        if (prevState != 2)
        {
            DB_PRINTLN("Auto Mixing Tank1 forceFill triggered.");
            prevState = 2;
        }
    }
    prevAutoMixing1 = autoMixing1;

    if (autoMixing1)
    {
        if (forceFillMix1 || !mixingLow1)
        {
            setMixingValve1(true);
            delay(200);
            setMixingPump(true); // Bơm sẽ bật nếu valve1 hoặc valve2 mở
            currentState = 1;
            if (currentState != prevState)
            {
                DB_PRINTLN("ForceFill or no water detected: Mixing Tank 1 Supply ON (auto mode).");
            }
        }
        else
        {
            setMixingValve1(false);
            setMixingPump(false); // Tắt bơm nếu không có van nào mở
            currentState = 0;
            if (currentState != prevState)
            {
                DB_PRINTLN("Mixing Tank 1 NORMAL (between): Mixing Supply OFF (auto mode).");
            }
        }
        prevState = currentState;
    }
    else
    {
        bool manualValve = Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_VAN_CHIA1);
        if (getMixingValve1() != manualValve)
            setMixingValve1(manualValve);
        setMixingPump(false); // Cập nhật trạng thái bơm dựa trên van
        prevState = -1;
    }
}

// Auto Mixing (Cấp Nước cho Bồn Pha) cho Tank2:
void Irrigation4Channel::autoMixingControl_Tank2()
{
    bool mixingLow2 = myFlowBoard->getFilteredInputValue(INPUT_MIXING_TANK2_LOW);
    bool mixingHigh2 = myFlowBoard->getFilteredInputValue(INPUT_MIXING_TANK2_HIGH);
    bool mixingHigh1 = myFlowBoard->getFilteredInputValue(INPUT_MIXING_TANK1_HIGH); // Kiểm tra bồn 1

    // Nếu cả hai bồn đều đầy, không cho phép mở van chia hoặc bơm
    if (mixingHigh1 && mixingHigh2)
    {
        setMixingValve2(false);
        setMixingPump(false); // Tắt bơm vì không có van nào cần mở
        DB_PRINTLN("Both Tanks FULL: Mixing Pump and Valve 2 forced OFF.");
        return;
    }

    // Nếu chỉ bồn 2 đầy, tắt van chia 2 và để bồn 1 quyết định bơm
    if (mixingHigh2)
    {
        setMixingValve2(false);
        setMixingPump(false); // Tắt tạm thời, bồn 1 sẽ kiểm tra lại
        DB_PRINTLN("Mixing Tank 2 FULL: Valve 2 forced OFF.");
        return;
    }

    bool autoMixing2 = Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_MIXING_TANK2);
    static bool prevAutoMixing2 = false;
    static bool forceFillMix2 = false;
    static int prevState = -1;
    int currentState = prevState;

    if (!prevAutoMixing2 && autoMixing2 && mixingLow2 && !mixingHigh2)
    {
        forceFillMix2 = true;
        if (prevState != 2)
        {
            DB_PRINTLN("Auto Mixing Tank2 forceFill triggered.");
            prevState = 2;
        }
    }
    prevAutoMixing2 = autoMixing2;

    if (autoMixing2)
    {
        if (forceFillMix2 || !mixingLow2)
        {
            setMixingValve2(true);
            delay(200);
            setMixingPump(true); // Bơm sẽ bật nếu valve1 hoặc valve2 mở
            currentState = 1;
            if (currentState != prevState)
            {
                DB_PRINTLN("ForceFill or no water detected: Mixing Tank 2 Supply ON (auto mode).");
            }
        }
        else
        {
            setMixingValve2(false);
            setMixingPump(false); // Tắt bơm nếu không có van nào mở
            currentState = 0;
            if (currentState != prevState)
            {
                DB_PRINTLN("Mixing Tank 2 NORMAL (between): Mixing Supply OFF (auto mode).");
            }
        }
        prevState = currentState;
    }
    else
    {
        bool manualValve = Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_VAN_CHIA2);
        if (getMixingValve2() != manualValve)
            setMixingValve2(manualValve);
        setMixingPump(false); // Cập nhật trạng thái bơm dựa trên van
        prevState = -1;
    }
}
// Auto Irrigation Control for Tank1 (Tưới từ Bồn Pha 1):
// Tưới chỉ được kích hoạt nếu bồn pha có đủ nước, tức là sensor mực thấp báo true.
void Irrigation4Channel::autoIrrigationControl_Tank1()
{
    bool mixingPresent = myFlowBoard->getFilteredInputValue(INPUT_MIXING_TANK1_LOW);
    bool autoIrrigation1 = Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_IRRIGATION_TANK1);
    // prevState: 0 = OFF, 1 = ON; -1 = uninitialized.
    static int prevState = -1;
    int currentState;

    if (autoIrrigation1)
    {
        if (!mixingPresent)
        {
            setIrrigationValve1(false);
            setIrrigationPump1(false);
            currentState = 0;
            if (currentState != prevState)
            {
                DB_PRINTLN("Tank1 Irrigation: Mixing water insufficient -> OFF (auto mode).");
            }
        }
        else
        {
            setIrrigationValve1(true);
            delay(200);
            setIrrigationPump1(true);
            currentState = 1;
            if (currentState != prevState)
            {
                DB_PRINTLN("Tank1 Irrigation: Sufficient water -> ON (auto mode).");
            }
        }
        prevState = currentState;
    }
    else
    {
        if (getIrrigationPump1() != Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_IRRIGATION_PUMP1))
            setIrrigationPump1(Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_IRRIGATION_PUMP1));
        if (getIrrigationValve1() != Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_IRRIGATION_VALVE1))
            setIrrigationValve1(Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_IRRIGATION_VALVE1));
        prevState = -1;
    }
}

// Auto Irrigation Control for Tank2 (Tưới từ Bồn Pha 2):
void Irrigation4Channel::autoIrrigationControl_Tank2()
{
    bool mixingPresent = myFlowBoard->getFilteredInputValue(INPUT_MIXING_TANK2_LOW);
    bool autoIrrigation2 = Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_IRRIGATION_TANK2);
    // prevState: 0 = OFF, 1 = ON; -1 = uninitialized.
    static int prevState = -1;
    int currentState;

    if (autoIrrigation2)
    {
        if (!mixingPresent)
        {
            setIrrigationValve2(false);
            setIrrigationPump2(false);
            currentState = 0;
            if (currentState != prevState)
            {
                DB_PRINTLN("Tank2 Irrigation: Mixing water insufficient -> OFF (auto mode).");
            }
        }
        else
        {
            setIrrigationValve2(true);
            delay(200);
            setIrrigationPump2(true);
            currentState = 1;
            if (currentState != prevState)
            {
                DB_PRINTLN("Tank2 Irrigation: Sufficient water -> ON (auto mode).");
            }
        }
        prevState = currentState;
    }
    else
    {
        if (getIrrigationPump2() != Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_IRRIGATION_PUMP2))
            setIrrigationPump2(Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_IRRIGATION_PUMP2));
        if (getIrrigationValve2() != Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_IRRIGATION_VALVE2))
            setIrrigationValve2(Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_IRRIGATION_VALVE2));
        prevState = -1;
    }
}

//---------------------------------------------------------------------
// Auto EC Dosing Control for Mixing Tank 1 with Protection
//---------------------------------------------------------------------
void Irrigation4Channel::autoECControl_Tank1(float currentEC, float setEC, unsigned long dosingTime, unsigned long waitTime, int ratioA, int ratioB)
{
    unsigned long timeDoseA = (unsigned long)((dosingTime * ratioA / 100.0) * 1000);
    unsigned long timeDoseB = (unsigned long)((dosingTime * ratioB / 100.0) * 1000);
    unsigned long waitDuration = waitTime * 1000;

    static unsigned long dosingStartTime = 0;
    static int dosingCycleCount = 0;
    static float lastEC = 0;
    static bool isDosing = false;
    static bool isWaiting = false;
    // prevPhase: 0 = idle, 1 = dosing, 2 = waiting, 3 = waiting complete, 4 = error; -1 = uninitialized.
    static int prevPhase = -1;

    bool autoEcTank1 = Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_EC_TANK1);

    if (!autoEcTank1)
    {
        bool manualPump1A = Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_FERT_PUMP1A);
        bool manualPump1B = Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_FERT_PUMP1B);

        if (getFertilizerPump1A() != manualPump1A)
            setFertilizerPump1A(manualPump1A);
        if (getFertilizerPump1B() != manualPump1B)
            setFertilizerPump1B(manualPump1B);

        isDosing = false;
        isWaiting = false;
        dosingCycleCount = 0;
        prevPhase = 0;
        return;
    }

    if (currentEC >= setEC)
    {
        if (getFertilizerPump1A())
            setFertilizerPump1A(false);
        if (getFertilizerPump1B())
            setFertilizerPump1B(false);
        isDosing = false;
        isWaiting = false;
        dosingCycleCount = 0;
        if (prevPhase != 0)
        {
            DB_PRINTLN("Tank1: EC above set point. Dosing stopped.");
            prevPhase = 0;
        }
        lastEC = currentEC;
        return;
    }

    if (!isDosing && !isWaiting)
    {
        dosingStartTime = millis();
        isDosing = true;
        dosingCycleCount++;
        setFertilizerPump1A(true);
        setFertilizerPump1B(true);
        if (prevPhase != 1)
        {
            DB_PRINTLN("Tank1: Starting EC dosing.");
            prevPhase = 1;
        }
        return;
    }

    if (isDosing)
    {
        unsigned long elapsed = millis() - dosingStartTime;
        if (elapsed >= timeDoseA)
            setFertilizerPump1A(false);
        if (elapsed >= timeDoseB)
            setFertilizerPump1B(false);
        if (elapsed >= max(timeDoseA, timeDoseB))
        {
            isDosing = false;
            isWaiting = true;
            dosingStartTime = millis();
            if (prevPhase != 2)
            {
                DB_PRINTLN("Tank1: EC dosing phase complete. Waiting...");
                prevPhase = 2;
            }
        }
    }

    if (isWaiting)
    {
        if (millis() - dosingStartTime >= waitDuration)
        {
            isWaiting = false;
            if (prevPhase != 3)
            {
                DB_PRINTLN("Tank1: Waiting complete. Ready for next dosing cycle.");
                prevPhase = 3;
            }
        }
        if (dosingCycleCount >= 10 && fabs(currentEC - lastEC) < 0.1)
        {
            Mlt16IoCommunication.mbTcp->coilWrite(COIL_AUTO_EC_TANK1, false);
            Mlt16IoCommunication.mbTcp->coilWrite(COIL_EC_ERROR1, true);
            if (prevPhase != 4)
            {
                DB_PRINTLN("Tank1: EC dosing error. Auto EC disabled.");
                prevPhase = 4;
            }
            isDosing = false;
            isWaiting = false;
            dosingCycleCount = 0;
            return;
        }
        lastEC = currentEC;
    }
}

//---------------------------------------------------------------------
// Auto EC Dosing Control for Mixing Tank 2 with Protection
//---------------------------------------------------------------------
void Irrigation4Channel::autoECControl_Tank2(float currentEC, float setEC, unsigned long dosingTime, unsigned long waitTime, int ratioA, int ratioB)
{
    unsigned long timeDoseA = (unsigned long)((dosingTime * ratioA / 100.0) * 1000);
    unsigned long timeDoseB = (unsigned long)((dosingTime * ratioB / 100.0) * 1000);
    unsigned long waitDuration = waitTime * 1000;

    static bool isDosing = false;
    static bool isWaiting = false;
    static unsigned long dosingStartTime = 0;
    static int dosingCycleCount = 0;
    static float lastEC = 0;
    // prevPhase: 0 = idle, 1 = dosing, 2 = waiting, 3 = waiting complete, 4 = error; -1 = uninitialized.
    static int prevPhase = -1;
    // Các biến để log trạng thái cho bơm A/B chỉ một lần.
    static bool pumpALogged = false;
    static bool pumpBLogged = false;

    bool autoEcTank2 = Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_EC_TANK2);

    if (!autoEcTank2)
    {
        bool manualPump2A = Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_FERT_PUMP2A);
        bool manualPump2B = Mlt16IoCommunication.mbTcp->coilRead(COIL_OUTPUT_FERT_PUMP2B);
        if (getFertilizerPump2A() != manualPump2A)
            setFertilizerPump2A(manualPump2A);
        if (getFertilizerPump2B() != manualPump2B)
            setFertilizerPump2B(manualPump2B);
        isDosing = false;
        isWaiting = false;
        dosingCycleCount = 0;
        prevPhase = 0;
        pumpALogged = false;
        pumpBLogged = false;
        return;
    }

    if (currentEC >= setEC)
    {
        if (isDosing || isWaiting)
        {
            if (prevPhase != 0)
            {
                DB_PRINTLN("Tank2: EC above set point. Stopping dosing.");
                prevPhase = 0;
            }
        }
        if (getFertilizerPump2A())
            setFertilizerPump2A(false);
        if (getFertilizerPump2B())
            setFertilizerPump2B(false);
        isDosing = false;
        isWaiting = false;
        dosingCycleCount = 0;
        lastEC = currentEC;
        pumpALogged = false;
        pumpBLogged = false;
        return;
    }

    if (!isDosing && !isWaiting)
    {
        dosingStartTime = millis();
        isDosing = true;
        dosingCycleCount++;
        setFertilizerPump2A(true);
        setFertilizerPump2B(true);
        pumpALogged = false;
        pumpBLogged = false;
        if (prevPhase != 1)
        {
            DB_PRINTLN("Tank2: Starting EC dosing cycle.");
            prevPhase = 1;
        }
        return;
    }

    if (isDosing)
    {
        unsigned long elapsed = millis() - dosingStartTime;
        if (elapsed >= timeDoseA && getFertilizerPump2A())
        {
            setFertilizerPump2A(false);
            if (!pumpALogged)
            {
                DB_PRINTLN("Tank2: Fertilizer Pump 2A dosing complete.");
                pumpALogged = true;
            }
        }
        if (elapsed >= timeDoseB && getFertilizerPump2B())
        {
            setFertilizerPump2B(false);
            if (!pumpBLogged)
            {
                DB_PRINTLN("Tank2: Fertilizer Pump 2B dosing complete.");
                pumpBLogged = true;
            }
        }
        if (!getFertilizerPump2A() && !getFertilizerPump2B())
        {
            isDosing = false;
            isWaiting = true;
            dosingStartTime = millis();
            if (prevPhase != 2)
            {
                DB_PRINTLN("Tank2: Dosing phase complete. Starting waiting phase.");
                prevPhase = 2;
            }
        }
        return;
    }

    if (isWaiting)
    {
        if (millis() - dosingStartTime >= waitDuration)
        {
            isWaiting = false;
            if (prevPhase != 3)
            {
                DB_PRINTLN("Tank2: Waiting phase complete. Ready for next cycle.");
                prevPhase = 3;
            }
        }
        if (dosingCycleCount >= 10 && fabs(currentEC - lastEC) < 0.1)
        {
            if (prevPhase != 4)
            {
                DB_PRINTLN("Tank2: EC dosing error: malfunction detected. Turning off auto EC dosing.");
                prevPhase = 4;
            }
            Mlt16IoCommunication.mbTcp->coilWrite(COIL_EC_ERROR2, true);
            isDosing = false;
            isWaiting = false;
            dosingCycleCount = 0;
            return;
        }
        lastEC = currentEC;
    }
}

// Hàm khởi tạo trạng thái ban đầu cho hệ thống
void Irrigation4Channel::init()
{
    // Tắt toàn bộ Output
    setWaterSupplyPump(false);
    setMixingValve1(false);
    setMixingValve2(false);
    setIrrigationPump1(false);
    setIrrigationValve1(false);
    setIrrigationPump2(false);
    setIrrigationValve2(false);
    setFertilizerPump1A(false);
    setFertilizerPump1B(false);
    setFertilizerPump2A(false);
    setFertilizerPump2B(false);

    // Tắt tất cả chế độ tự động
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_AUTO_WATER_SUPPLY, false);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_AUTO_MIXING_TANK1, false);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_AUTO_MIXING_TANK2, false);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_AUTO_IRRIGATION_TANK1, false);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_AUTO_IRRIGATION_TANK2, false);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_AUTO_EC_TANK1, false);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_AUTO_EC_TANK2, false);

    // Reset các coil báo lỗi EC dosing
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_EC_ERROR1, false);
    Mlt16IoCommunication.mbTcp->coilWrite(COIL_EC_ERROR2, false);

    DB_PRINTLN("All devices, auto modes, errors and internal states are reset at startup.");
}

// Hàm cập nhật và in giá trị mỗi 1 giây (debug)
void Irrigation4Channel::updateEvery1s()
{
    // Nếu không dùng, bạn có thể comment hàm này lại

    DB_PRINTLN("====== System status update ======");

    for (int i = 0; i < 8; i++)
    {
        bool sensorStatus = myFlowBoard->getFilteredInputValue(i);

        DB_PRINT("Current input sensor ");
        DB_PRINT(i);
        DB_PRINT(": ");
        DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_INPUT_STATUS(i)) ? "ON" : "OFF");
    }

    // In trạng thái EC sensors
    DB_PRINT("Current EC Sensor1: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->inputRegisterRead(INPUT_SENSOR1_EC));
    DB_PRINT("Current PH Sensor1: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->inputRegisterRead(INPUT_SENSOR1_PH));
    DB_PRINT("Current TEMP Sensor1: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->inputRegisterRead(INPUT_SENSOR1_TEMP));
    DB_PRINT("Current EC Sensor2: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->inputRegisterRead(INPUT_SENSOR2_EC));
    DB_PRINT("Current PH Sensor2: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->inputRegisterRead(INPUT_SENSOR2_PH));
    DB_PRINT("Current TEMP Sensor2: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->inputRegisterRead(INPUT_SENSOR2_TEMP));
    // In trạng thái Output coils
    DB_PRINTLN("-- Output status --");
    for (int i = COIL_OUTPUT_MIXING_PUMP; i <= COIL_OUTPUT_IRRIGATION_VALVE2; i++)
    {
        DB_PRINT("Output Coil ");
        DB_PRINT(i);
        DB_PRINT(": ");
        DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(i) ? "ON" : "OFF");
    }

    // In trạng thái các coil auto mode
    DB_PRINT("Auto Water Supply: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_WATER_SUPPLY));

    DB_PRINT("Auto Mixing Tank1: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_MIXING_TANK1));

    DB_PRINT("Auto Mixing Tank2: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_MIXING_TANK2));

    DB_PRINT("Auto Irrigation Tank1: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_IRRIGATION_TANK1));

    DB_PRINT("Auto Irrigation Tank2: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_IRRIGATION_TANK2));

    DB_PRINT("Auto EC Tank1: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_EC_TANK1));

    DB_PRINT("Auto EC Tank2: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_AUTO_EC_TANK2));

    // Các flag lỗi EC
    DB_PRINT("EC Error Tank1: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_EC_ERROR1) ? "YES" : "NO");

    DB_PRINT("EC Error Tank2: ");
    DB_PRINTLN(Mlt16IoCommunication.mbTcp->coilRead(COIL_EC_ERROR2) ? "YES" : "NO");

    DB_PRINTLN("==================================");
}
