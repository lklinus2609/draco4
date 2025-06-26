/** \file
 * \brief Velocity control test for SOEM EtherCAT master
 *
 * Based on simple_test, modified for velocity control with correct PDO mapping
 * Usage: velocity_test [ifname]
 */
// enx3c18a042f97c port number

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <signal.h>

#include <soem/ethercat.h>

#define EC_TIMEOUTMON 500

// ========== VELOCITY CONTROL PARAMETERS ==========
// Easy to modify for testing different velocities
#define TARGET_VELOCITY_RPM    -1000   // Change this to test different speeds
#define MAX_VELOCITY_RPM      1000   // Safety limit
#define VELOCITY_MODE         0x09   // Profile velocity mode

// Test presets
#define TEST_SLOW      50
#define TEST_MEDIUM   200
#define TEST_FAST     500

// Conversion factors (adjust based on your motor)
#define COUNTS_PER_REV        4096   // Encoder counts per revolution
#define VELOCITY_FACTOR       10     // Motor-specific velocity scaling

// ========== CIA 402 CONSTANTS ==========
#define CONTROLWORD_SHUTDOWN           0x0006
#define CONTROLWORD_SWITCH_ON          0x0007
#define CONTROLWORD_ENABLE_OPERATION   0x000F
#define CONTROLWORD_DISABLE_VOLTAGE    0x0000
#define CONTROLWORD_QUICK_STOP         0x0002
#define CONTROLWORD_FAULT_RESET        0x0080

#define STATUSWORD_READY_TO_SWITCH_ON  0x0021
#define STATUSWORD_SWITCHED_ON         0x0023
#define STATUSWORD_OPERATION_ENABLED   0x0237
#define STATUSWORD_FAULT              0x0008
#define STATUSWORD_VOLTAGE_ENABLED     0x0010

// ========== PDO STRUCTURES ==========
// Output PDO: 35 bytes total (matches exact mapping)
typedef struct {
    uint16_t controlword;          // [0x0000] Bytes 0-1
    uint8_t  modes_of_operation;   // [0x0002] Byte 2  
    uint16_t target_torque;        // [0x0003] Bytes 3-4
    uint32_t target_position;      // [0x0005] Bytes 5-8
    uint32_t target_velocity;      // [0x0009] Bytes 9-12 ← KEY FIELD!
    uint16_t torque_offset;        // [0x000D] Bytes 13-14
    uint32_t tuning_command;       // [0x000F] Bytes 15-18
    uint32_t physical_outputs;     // [0x0013] Bytes 19-22
    uint32_t bit_mask;            // [0x0017] Bytes 23-26
    uint32_t user_mosi;           // [0x001B] Bytes 27-30
    uint32_t velocity_offset;     // [0x001F] Bytes 31-34
} __attribute__((packed)) motor_outputs_t;

// Input PDO: 47 bytes total
typedef struct {
    uint16_t statusword;                    // [0x0023] Bytes 0-1
    uint8_t  modes_of_operation_display;    // [0x0025] Byte 2
    uint32_t position_actual;               // [0x0026] Bytes 3-6
    uint32_t velocity_actual;               // [0x002A] Bytes 7-10
    uint16_t torque_actual;                 // [0x002E] Bytes 11-12
    uint32_t position_following_error;      // [0x0030] Bytes 13-16
    uint32_t user_miso;                     // [0x0034] Bytes 17-20
    uint32_t digital_inputs;                // [0x0038] Bytes 21-24
    uint8_t  reserved[22];                  // Padding to 47 bytes
} __attribute__((packed)) motor_inputs_t;

// ========== GLOBAL VARIABLES ==========
char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

// Motor control specific
volatile boolean motor_enabled = FALSE;
volatile boolean running = TRUE;
int32_t current_velocity_rpm = TARGET_VELOCITY_RPM;

// ========== UTILITY FUNCTIONS ==========
// Convert RPM to motor counts
int32_t rpm_to_counts(int rpm) {
    return (rpm * COUNTS_PER_REV * VELOCITY_FACTOR) / 60;
}

// Convert motor counts to RPM
int rpm_from_counts(int32_t counts) {
    return (counts * 60) / (COUNTS_PER_REV * VELOCITY_FACTOR);
}

// Signal handler for clean shutdown
void signal_handler(int sig) {
    (void)sig;  // Suppress unused parameter warning
    printf("\nShutting down...\n");
    running = FALSE;
}

// ========== MOTOR CONTROL FUNCTIONS ==========
// Get motor state string
const char* get_motor_state(uint16_t statusword) {
    if (statusword & STATUSWORD_FAULT)
        return "FAULT";
    if ((statusword & 0x006F) == 0x0027)  // STATUSWORD_OPERATION_ENABLED = 0x0237, masked with 0x006F = 0x0027
        return "OPERATION ENABLED";
    if ((statusword & 0x006F) == STATUSWORD_SWITCHED_ON)
        return "SWITCHED ON";
    if ((statusword & 0x006F) == STATUSWORD_READY_TO_SWITCH_ON)
        return "READY TO SWITCH ON";
    if ((statusword & 0x004F) == 0x0040)
        return "SWITCH ON DISABLED";
    return "UNKNOWN";
}

// Enable motor using CIA402 state machine
int enable_motor_sequence(motor_outputs_t *outputs, motor_inputs_t *inputs, int step) {
    uint16_t status = inputs->statusword;
    static int cycle_count = 0;
    cycle_count++;
    
    switch(step) {
        case 0:  // Fault reset if needed
            if (status & STATUSWORD_FAULT) {
                outputs->controlword = CONTROLWORD_FAULT_RESET;
                printf("Resetting fault...\n");
                return 0;
            }
            return 1;
            
        case 1:  // Shutdown
            outputs->controlword = CONTROLWORD_SHUTDOWN;
            if ((status & 0x006F) == STATUSWORD_READY_TO_SWITCH_ON) {
                printf("Motor ready to switch on\n");
                return 2;
            }
            return 1;
            
        case 2:  // Switch on
            outputs->controlword = CONTROLWORD_SWITCH_ON;
            outputs->modes_of_operation = VELOCITY_MODE;  // Set velocity mode early!
            if ((status & 0x006F) == STATUSWORD_SWITCHED_ON) {
                printf("Motor switched on, velocity mode set\n");
                return 3;
            }
            return 2;
            
        case 3:  // Enable operation
            outputs->controlword = CONTROLWORD_ENABLE_OPERATION;
            outputs->modes_of_operation = VELOCITY_MODE;  // Keep velocity mode set
            
            // Debug: print raw statusword to see why it's not transitioning
            if ((cycle_count % 50) == 0) {  // Print every 50 cycles to avoid spam
                printf("Debug - Statusword: 0x%04X, Mode display: %d\n", 
                       status, inputs->modes_of_operation_display);
            }
            
            if ((status & 0x006F) == 0x0027) {  // STATUSWORD_OPERATION_ENABLED masked with 0x006F = 0x0027
                printf("Motor operation enabled!\n");
                motor_enabled = TRUE;
                return 4;
            }
            return 3;
            
        default:
            return 4;  // Done
    }
}

// ========== MAIN VELOCITY TEST FUNCTION ==========
void velocity_test(char *ifname)
{
    int i, oloop, iloop, chk;
    int enable_step = 0;
    uint32_t last_position = 0;
    int print_counter = 0;
    
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting velocity control test\n");
    printf("Target velocity: %d RPM\n", current_velocity_rpm);
    printf("Press Ctrl+C to stop\n\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        
        /* find and auto-config slaves */
        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);
            
            // Verify PDO mapping after config_map
            ec_config_map(&IOmap);
            ec_configdc();
            
            printf("\nPDO mapping configured - Output bytes: %d, Input bytes: %d\n", 
                   ec_slave[1].Obytes, ec_slave[1].Ibytes);

            printf("\nSlaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 8) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 8) iloop = 8;

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n\n");
                inOP = TRUE;
                
                // Get pointers to PDO data
                motor_outputs_t *outputs = (motor_outputs_t *)ec_slave[1].outputs;
                motor_inputs_t *inputs = (motor_inputs_t *)ec_slave[1].inputs;
                
                // Initialize outputs
                memset(outputs, 0, sizeof(motor_outputs_t));
                
                printf("Starting velocity control loop...\n");
                printf("===========================================\n");
                
                /* cyclic loop */
                for(i = 1; running && i <= 100000; i++)
                {
                    // Update process data
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if(wkc >= expectedWKC)
                    {
                        // Motor enable sequence (first 250 cycles, equivalent to 1 second at 250Hz)
                        if (!motor_enabled && i < 250) {
                            if (i % 3 == 0) {  // Run enable sequence every 3 cycles (~12ms at 250Hz)
                                enable_step = enable_motor_sequence(outputs, inputs, enable_step);
                            }
                        }
                        
                        // Set velocity mode and command after motor is enabled
                        if (motor_enabled) {
                            outputs->modes_of_operation = VELOCITY_MODE;
                            outputs->target_velocity = rpm_to_counts(current_velocity_rpm);
                            outputs->controlword = CONTROLWORD_ENABLE_OPERATION;  // Keep enabled
                        }
                        
                        // Print status every 25 cycles (100ms at 4ms cycle time)
                        if (++print_counter >= 25) {
                            print_counter = 0;
                            
                            int32_t actual_rpm = rpm_from_counts(inputs->velocity_actual);
                            int32_t position_change = inputs->position_actual - last_position;
                            last_position = inputs->position_actual;
                            
                            printf("\rCycle %6d | State: %-20s | Cmd: %4d RPM | Act: %4d RPM | Pos: %8d | ΔPos: %6d",
                                   i, 
                                   get_motor_state(inputs->statusword),
                                   current_velocity_rpm,
                                   actual_rpm,
                                   inputs->position_actual,
                                   position_change);
                            fflush(stdout);
                            
                            // Check for motion
                            if (motor_enabled && abs(position_change) > 10) {
                                needlf = TRUE;  // Motor is moving
                            }
                        }
                    }
                    else
                    {
                        printf("\nWorkcounter error: expected %d, got %d\n", expectedWKC, wkc);
                    }
                    
                    osal_usleep(4000);  // 4ms cycle time for 250Hz operation
                }
                
                // Shutdown sequence
                printf("\n\nShutting down motor...\n");
                outputs->target_velocity = 0;
                outputs->controlword = CONTROLWORD_SHUTDOWN;
                
                for(i = 0; i < 100; i++) {
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    osal_usleep(4000);  // 4ms cycle time for 250Hz operation
                }
                
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, 
                            ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End velocity test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExecute as root\n",ifname);
    }
}

// ========== ERROR CHECKING THREAD ==========
OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(running)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

// ========== MAIN FUNCTION ==========
int main(int argc, char *argv[])
{
    printf("SOEM Velocity Control Test\n");
    printf("==========================\n\n");
    
    // Install signal handler for clean shutdown
    signal(SIGINT, signal_handler);

    if (argc > 1)
    {
        // Optional: parse velocity from command line
        if (argc > 2) {
            int vel = atoi(argv[2]);
            if (vel > 0 && vel <= MAX_VELOCITY_RPM) {
                current_velocity_rpm = vel;
            } else {
                printf("Velocity must be between 1 and %d RPM\n", MAX_VELOCITY_RPM);
                return 1;
            }
        }
        
        /* create thread to handle slave error handling in OP */
        osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
        
        /* start velocity test */
        velocity_test(argv[1]);
    }
    else
    {
        ec_adaptert * adapter = NULL;
        printf("Usage: velocity_test ifname [velocity_rpm]\n");
        printf("  ifname = network interface (e.g., eth0)\n");
        printf("  velocity_rpm = target velocity (optional, default: %d)\n\n", TARGET_VELOCITY_RPM);
        
        printf("Available adapters:\n");
        adapter = ec_find_adapters();
        while (adapter != NULL)
        {
            printf("    - %s  (%s)\n", adapter->name, adapter->desc);
            adapter = adapter->next;
        }
    }

    printf("\nProgram terminated.\n");
    return 0;
}