#include "vex.h"
#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <algorithm>

// --------------------- VEX Setup ---------------------
using namespace vex;
vex::brain Brain;
vex::motor intake(PORT7); // TODO: intake doesn't exist
vex::motor scoring(PORT11);
vex::motor clamp(PORT16);
vex::motor rightFront(PORT18);
vex::motor rightBack(PORT17);
vex::motor leftFront(PORT19, true);
vex::motor leftBack(PORT20, true);

// Sensors
vex::gps gpsSensorLeft(PORT9);
vex::gps gpsSensorRight(PORT8);
vex::inertial inersense(PORT10);

// --------------------- Enums -------------------------
// These are the same "Special" bytes as Pi→VEX uses.
static constexpr uint8_t START  = 0xAA;
static constexpr uint8_t END    = 0xAB;
static constexpr uint8_t ESCAPE = 0xAC;

// Example "Instruction" enum (for Pi→VEX commands):
enum class Instruction : uint8_t {
  LF_SET     = 0x01,
  RF_SET     = 0x02,
  LB_SET     = 0x03,
  RB_SET     = 0x04,
  MOTOR_SET  = 0x05,
  INTAKE_SET = 0x06,
  DRIVE_SET  = 0x07
};

// Data Tags for VEX→Pi
enum class DataTag : uint8_t {
    GPS0_X = 0x01, // m
    GPS0_Y = 0x02, // m
    GPS0_H = 0x03, // degrees
    GPS1_X = 0x04, // m
    GPS1_Y = 0x05, // m
    GPS1_H = 0x06, // degrees

    GYRO = 0x07, // degrees

    FL = 0x0B,
    FR = 0x0C,
    RL = 0x0D,
    RR = 0x0E,
  };

  // Clamp Logic
void zeroClamp() {
    // Spin CW until velocity is < 5% for 0.5s
    clamp.setVelocity(100, percent);
    clamp.spin(forward);
    while (clamp.velocity(percentUnits::pct) > 5) {
        vex::this_thread::sleep_for(20);
    }
    clamp.stop();
    clamp.setPosition(0, degrees);
}

void engageClamp() {
    // Spin CCW until velocity is < 5% and encoder is greaten than threshold (if vel = 0 before encoder then zero instead)
    clamp.setVelocity(100, percent);
    clamp.spin(reverse);
    while (clamp.velocity(percentUnits::pct) > 5) {
        vex::this_thread::sleep_for(20);
    }

}

// --------------------- Handling Pi→VEX --------------------
void setDrive(double vel1, double vel2, double vel3, double vel4) {
    leftFront.setVelocity(vel1, percent);
    leftBack.setVelocity(vel2, percent);
    rightFront.setVelocity(vel3, percent);
    rightBack.setVelocity(vel4, percent);

    leftFront.spin(forward);
    leftBack.spin(forward);
    rightFront.spin(forward);
    rightBack.spin(forward);
}

// Motor handlers from your original code
void handleLFSet(const std::vector<double>& operands);
void handleRFSet(const std::vector<double>& operands);
void handleLBSet(const std::vector<double>& operands);
void handleRBSet(const std::vector<double>& operands);
void handleMotorSet(const std::vector<double>& operands);
void handleIntakeSet(const std::vector<double>& operands);
void handleDriveSet(const std::vector<double>& operands);

// Decoding Pi→VEX messages: same special bytes, same decode.
std::vector<double> decodeOperands(const std::vector<uint8_t>& operandBytes) {
    std::vector<double> decodedDoubles;
    std::vector<uint8_t> buffer;
    bool escapeNext = false;

    for (uint8_t byte : operandBytes) {
        if (escapeNext) {
            if (byte == 0x00)
                buffer.push_back(START);
            else if (byte == 0x01)
                buffer.push_back(END);
            else if (byte == 0x02)
                buffer.push_back(ESCAPE);
            escapeNext = false;
        } else if (byte == ESCAPE) {
            escapeNext = true;
        } else {
            buffer.push_back(byte);
        }

        if (buffer.size() == sizeof(double)) {
            // Big-endian to host order:
            std::reverse(buffer.begin(), buffer.end());
            double value;
            std::memcpy(&value, buffer.data(), sizeof(double));
            decodedDoubles.push_back(value);
            buffer.clear();
        }
    }
    return decodedDoubles;
}

void decodeMessage(const std::vector<uint8_t>& message) {
    if (message.size() < 2) return;
    if (message[0] != START) return;

    Instruction instruction = static_cast<Instruction>(message[1]);
    std::vector<uint8_t> operandBytes(message.begin() + 2, message.end());
    std::vector<double> decodedOperands = decodeOperands(operandBytes);

    switch (instruction) {
      case Instruction::LF_SET:      handleLFSet(decodedOperands); break;
      case Instruction::RF_SET:      handleRFSet(decodedOperands); break;
      case Instruction::LB_SET:      handleLBSet(decodedOperands); break;
      case Instruction::RB_SET:      handleRBSet(decodedOperands); break;
      case Instruction::MOTOR_SET:   handleMotorSet(decodedOperands); break;
      case Instruction::INTAKE_SET:  handleIntakeSet(decodedOperands); break;
      case Instruction::DRIVE_SET:   handleDriveSet(decodedOperands); break;
      default: /* ignore unknown IDs */ break;
    }
}

// --------------------- Pi→VEX Handlers --------------------
int count = 0;
void handleLFSet(const std::vector<double>& operands) {
    leftFront.setVelocity(operands[0], percent);
    leftFront.spin(forward);
    count++;
}
void handleRFSet(const std::vector<double>& operands) {
    rightFront.setVelocity(operands[0], percent);
    rightFront.spin(forward);
    count++;
}
void handleLBSet(const std::vector<double>& operands) {
    leftBack.setVelocity(operands[0], percent);
    leftBack.spin(forward);
    count++;
}
void handleRBSet(const std::vector<double>& operands) {
    rightBack.setVelocity(operands[0], percent);
    rightBack.spin(forward);
    count++;
}

void handleMotorSet(const std::vector<double>& operands) {
    if (operands.size() < 2) return;
    int motorID = static_cast<int>(operands[0]);
    double speed = operands[1];

    switch (motorID) {
        case 1: leftFront.setVelocity(speed, percent); leftFront.spin(forward); break;
        case 2: rightFront.setVelocity(speed, percent); rightFront.spin(forward); break;
        case 3: leftBack.setVelocity(speed, percent); leftBack.spin(forward); break;
        case 4: rightBack.setVelocity(speed, percent); rightBack.spin(forward); break;
        default: break; // Ignore unknown motor IDs
    }
}

void handleIntakeSet(const std::vector<double>& operands) {
    intake.spin(forward, operands[0], percent);
}
void handleDriveSet(const std::vector<double>& operands) {
    if (operands.size() < 4) return;
    double lf = operands[0];
    double rf = operands[1];
    double lb = operands[2];
    double rb = operands[3];
    setDrive(lf, lb, rf, rb);
}

// --------------------- NEW: VEX→Pi Packet Encoding ---------------------

// 1) Build a double in big-endian form
std::vector<uint8_t> buildDoubleBytes(double value) {
    // Convert host double to big-endian bytes
    std::vector<uint8_t> raw(sizeof(double));
    std::memcpy(raw.data(), &value, sizeof(double));
    // Our decode expects big-endian, so reverse it
    std::reverse(raw.begin(), raw.end());
    return raw;
}

// 2) Escape special bytes (START, END, ESCAPE) so we don't break our packets
void escapeAndAppend(std::vector<uint8_t>& packet, const std::vector<uint8_t>& data) {
    for (uint8_t b : data) {
        if (b == START) {
            packet.push_back(ESCAPE);
            packet.push_back(0x00);
        } else if (b == END) {
            packet.push_back(ESCAPE);
            packet.push_back(0x01);
        } else if (b == ESCAPE) {
            packet.push_back(ESCAPE);
            packet.push_back(0x02);
        } else {
            packet.push_back(b);
        }
    }
}

// 3) Build a single packet with multiple (Tag, double-value) pairs
//    e.g. encodeDataPacket({{DataTag::GPS_X, 123.45}, {DataTag::GPS_Y, 67.89}, ...})
std::vector<uint8_t> encodeDataPacket(const std::vector<std::pair<DataTag, double>>& dataPairs) {
    std::vector<uint8_t> packet;
    packet.push_back(START);

    // For each pair, add the tag, then the escaped double
    for (auto& p : dataPairs) {
        // Tag
        packet.push_back(static_cast<uint8_t>(p.first));
        // Double
        auto doubleBytes = buildDoubleBytes(p.second);
        escapeAndAppend(packet, doubleBytes);
    }

    packet.push_back(END);
    return packet;
}

// 4) Task to periodically send the sensor data via the above packet structure
int sensorTask() {                     // ← renamed – launch this one below
    // 2 s for GPS, then wait for inertial calibration
    engageClamp();
    inersense.calibrate();
    while (inersense.isCalibrating()) {
        vex::this_thread::sleep_for(20);
    }

    // Zero the clamp then engage after 2 seconds

    while (true) {
        /* ---------- GPS ---------- */
        double xPos0 = gpsSensorLeft.xPosition(distanceUnits::mm) * 1000;
        double yPos0 = gpsSensorLeft.yPosition(distanceUnits::mm) * 1000;
        double xPos1 = gpsSensorRight.xPosition(distanceUnits::mm) * 1000;
        double yPos1 = gpsSensorRight.yPosition(distanceUnits::mm) * 1000;

        /* ---------- IMU ---------- */
        double gheading = inersense.heading(rotationUnits::deg) / 180 * 3.1415; // radians
        Brain.Screen.printAt(10, 10, "Heading: %f", gheading);

        /* ----- Wheel Encoders -----*/
        double FL = leftFront.velocity(percentUnits::pct) / 100 * 0.73333333333; // m/s ->  %/100 * (rpm * wheel travel AKA max speed)
        double FR = rightFront.velocity(percentUnits::pct) / 100 * 0.73333333333;
        double RL = leftBack.velocity(percentUnits::pct) / 100 * 0.73333333333;
        double RR = rightBack.velocity(percentUnits::pct) / 100 * 0.73333333333;

        /* ---------- Packet ---------- */
        std::vector<std::pair<DataTag, double>> data{
            // { DataTag::GPS0_X,       xPos0     },
            // { DataTag::GPS1_X,       xPos1       },
            // { DataTag::GPS0_Y,       yPos0       },
            // { DataTag::GPS1_Y,       yPos1       },
            // { DataTag::GPS0_H, gpsSensorLeft.heading() },
            // { DataTag::GPS1_H, gpsSensorRight.heading() },

            { DataTag::GYRO,         gheading        },

            // { DataTag::FL,          FL           },
            // { DataTag::FR,          FR           },
            // { DataTag::RL,          RL           },
            // { DataTag::RR,          RR           }
        };

        auto packet = encodeDataPacket(data);
        std::cout.write(reinterpret_cast<const char*>(packet.data()), packet.size());
        std::cout.flush();          // <‑‑ optional

        vex::this_thread::sleep_for(20);   // 50 Hz
    }
    return 0;
}


// --------------------- Main ---------------------
int main() {
    // Start a thread to send GPS data continuously
    vex::thread sensorThread(sensorTask);

    std::vector<uint8_t> message;
    char byte;
    bool readingMessage = false;

    rightFront.setBrake(brakeType::hold);
    rightBack.setBrake(brakeType::hold);
    leftFront.setBrake(brakeType::hold);
    leftBack.setBrake(brakeType::hold);

    // Main loop to decode Pi→VEX commands
    while (true) {
        intake.setVelocity(100, percent);
        intake.spin(forward);
        scoring.setVelocity(100, percent);
        scoring.spin(forward);
        // If there's a byte available from Pi->VEX, read it
        if (std::cin.read(&byte, 1)) {
            uint8_t currentByte = static_cast<uint8_t>(byte);

            if (currentByte == START) {
                message.clear();
                message.push_back(currentByte);
                readingMessage = true;
            }
            else if (currentByte == END && readingMessage) {
                message.push_back(currentByte);
                decodeMessage(message);
                readingMessage = false;
            }
            else if (readingMessage) {
                message.push_back(currentByte);
            }
        }
    }

    return 0;
}