#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include "joystick.h"
#include "robot_defines.h"
#include "easy_serial.h"
#include "360_controller_map.h"
#include "sensors.h"
#include "cobs.h"

#include <signal.h>
#include <sys/time.h>
#include <sys/types.h>
#include <linux/ipc.h>
#include <linux/msg.h>
#include "xv11.h"

// for thread
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
//--

//#define TELEGA
#define NEATO
#define LIDAR

#define DEBUG_JOYSTICK
#define VACUUM_COMM_LOCATION "/dev/ttyACM0" // USB0" // ACM3" //USB0" //ACM3"//USB0" //ACM1"  // temp
const char lidar_port[] = "/dev/ttyUSB0"; //ACM3";
const char joy_address[32] = "/dev/input/js0";

int  joy_file, received, send_port, receive_port; // file IDs for the joy, in serial, and out serial ports

#define ERROR_CREATE_THREAD -11
#define ERROR_JOIN_THREAD   -12

#ifdef NEATO

int lidar_to_use_flag = 1;

int N_MAX_SPEED = 300;

int buffer[10000] = {0};
#define DEADZONE_N            0.05f
char n_drive_cmd[30] = {};
char Neato_help[] = "help\n";
char Neato_Testmode_On[] = "TestMode On\n";
char Neato_Testmode_Off[] = "TestMode Off\n";
char Neato_drive_forward[] = "SetMotor 100 100 100\n";
char Neato_drive_backward[] = "SetMotor -100 -100 100\n";
char Neato_drive_left[] = "SetMotor -200 200 100\n";
char Neato_drive_right[] = "SetMotor 200 -200 100\n";
char Neato_drive_stop[] = "SetMotor 1 1 1\n";

char Neato_forward_var_speed[] = "SetMotor 100 100 ";
char Neato_backward_var_speed[] = "SetMotor -100 -100 ";
char Neato_left_var_speed[] = "SetMotor -200 200 ";
char Neato_right_var_speed[] = "SetMotor 200 -200 ";
float speed_val = 0.3;

char PlaySound_1[]={"PlaySound 1\n"};
char PlaySound_2[]={"PlaySound 2\n"};
char PlaySound_3[]={"PlaySound 3\n"};
char PlaySound_8[]={"PlaySound 8\n"};
char PlaySound_9[]={"PlaySound 9\n"};
char PlaySound_10[]={"PlaySound 10\n"};

int path_speed = 0;
int path_left = 100;
int path_right = 100;


char code_cmd = 100;
// Timer
static volatile sig_atomic_t g_called;
int flag_neato_stop = 0;
//end of Timer
#endif

int Lidar_forward_stop_flag = 1;


void init_serial_port(int tty_fd);

int send_command(unsigned char *flag, unsigned char *value, int send_port);

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float map_stick(short input)
{
    long temp = map(input, SHRT_MIN, SHRT_MAX, 100, -100);
    return (float)temp / 100;
}

void button_update(struct js_event *jse, int *button_update_array)
{
    button_update_array[jse->number] = jse->value;
    return;
}

void axis_update(struct js_event *jse, int *axis_update_array)
{
    axis_update_array[jse->number] = jse->value;
/*  static int count = 0;
     if (jse->number == 1) {
        printf("%d", axis_update_array[jse->number]);
        printf("\n");
        buffer[count] = jse->value;
        count ++;
    }*/
    return;
}

int  send_button_updates(int *old_button_array, int *new_button_array, int send_port, int receive_port)
{
    int i = 0;
    for(i = 0; i < BUTTON_COUNT; i++)
        if(new_button_array[i] != old_button_array[i])
        {
            update_button(i, new_button_array[i], send_port, receive_port);
            old_button_array[i] = new_button_array[i];
        }
    return 0;
}

int send_command_cobs(float value1, float value2, int send_port)
{
    char val_to_send[9];
    char val_cobs_to_send[11];
    memset(val_cobs_to_send, 0, 11);
    val_to_send[0] = 'u';
    memcpy(val_to_send + sizeof(char), &value1, sizeof(float));
    memcpy(val_to_send + sizeof(char) + sizeof(float), &value2, sizeof(float));

    cobs_encode((uint8_t *)val_to_send, sizeof(val_to_send), (uint8_t *)val_cobs_to_send);
    clearPort(send_port);
    write(send_port, val_cobs_to_send, 11);
    return 0;
}


int  send_axis_updates(int *old_axis_array, int *new_axis_array, int send_port)
{
    int i;
    int changed = 0;
    //printf("\nfork  speed %d path_left %d path_right %d  \n", path_speed, path_left, path_right);
    for(i = 0; i < AXIS_COUNT; i++)
    {
        if (new_axis_array[i] != old_axis_array[i]) {
            //update_axis(i, new_axis_array[i], send_port);
            changed = 1;
            old_axis_array[i] = new_axis_array[i];
        }
        if (changed)
        {
            float axis1, axis2;
            axis1 = map_stick(new_axis_array[AXIS_LEFT_STICK_VERTICAL]);
            axis2 = -map_stick(new_axis_array[AXIS_RIGHT_STICK_HORIZONTAL]);
            //keeps stuff in the deadzone from being sent.
/*            if(axis1 < DEADZONE && axis1 > -DEADZONE)
                axis1 = 0.0f;
            if(axis2 < DEADZONE && axis2 > -DEADZONE)
                axis2 = 0.0f;*/
#ifdef TELEGA
            printf("linear speed: %f, angular speed: %f\n", axis1, axis2);
            send_command_cobs(axis1, axis2, send_port);
#endif
#ifdef NEATO
///            printf("axis1 speed: %f, axis2 speed: %f\n", axis1, axis2);

            if (axis1>=DEADZONE) {   // forward
                if (axis2 >= DEADZONE) {   // right
                    path_left = 100;
                    path_right = 100 - 100 * axis2;
                }
                if (axis2 <= -DEADZONE) {  // left
                    path_left = 100+ 100 * axis2;
                    path_right = 100;
                }
                if ((axis2 <= DEADZONE) && (axis2 >= -DEADZONE)) {  // only forward
                    path_left = 100;
                    path_right = 100;
                }
                path_speed = N_MAX_SPEED*axis1;
            }
            if ((axis1 <= DEADZONE) && (axis1 >= -DEADZONE)) {  // turn left, right without moving forward, backward
                if (axis2 >= DEADZONE) {   // right
                    path_left =   100;
                    path_right = -100;
                    path_speed = N_MAX_SPEED*axis2;
                } else if (axis2 <= -DEADZONE) {  // left
                    path_left = -100;
                    path_right = 100;
                    path_speed = -N_MAX_SPEED*axis2;
                } else  {
                    path_left =  1; // !
                    path_right = 1; // !
                    path_speed = 1;
                }
            }
            if (axis1<=-DEADZONE) {  // backward
                if (axis2 >= DEADZONE) {   // right
                    path_left = -100;
                    path_right = -100 + 100 * axis2;
                }
                if (axis2 <= -DEADZONE) {  // left
                    path_left = -100 - 100 * axis2;
                    path_right = -100;
                }
                if ((axis2 < DEADZONE) && (axis2 > -DEADZONE)) {
                    path_right = -100;
                    path_left = -100;
                }
                path_speed = - N_MAX_SPEED * axis1;
            }
#endif
        }
    }
    return 0;
}

int make_cmd_string(char *drive_cmd, int Path_left, int Path_right, int Path_speed)
{
    int str_length = 0;
    char speed_val_string[5] ={};
    strcpy(drive_cmd, "SetMotor ");
    sprintf(speed_val_string, "%d", Path_left);
    strcat(drive_cmd, speed_val_string);
    strcat(drive_cmd, " ");
    sprintf(speed_val_string, "%d", Path_right);
    strcat(drive_cmd, speed_val_string);
    strcat(drive_cmd, " ");
    sprintf(speed_val_string, "%d", Path_speed);
    strcat(drive_cmd, speed_val_string);
    strcat(drive_cmd, "\r");
    str_length = strlen(drive_cmd);
    return str_length;
}

int send_command(unsigned char *flag, unsigned char *value, int send_port)
{
    /*
	unsigned char received = 0;
	clearPort(send_port);
	write(send_port, flag, 1);
	write(send_port, value, 1);
    //	while(received != ARDUINO_RECEIVED_BYTE);
    //	read(receive_port, &received, 1);
    */
     return 0;
}

int update_button(int button, int button_state, int send_port, int receive_port)
{
    unsigned char flag = 0;
    char value = 0;
    char buffer[512] = "";

    switch(button)
    {
        case BUTTON_LEFT_BUMPER:
            flag = LEFT_MOTOR_FLAG;
            if(button_state == 1)
            {
                value = 255;
                printf("Driving left motor.\n");
                send_command(&flag, &value, send_port);//do left bumper pressed thing
                clearPort(send_port);
                write(send_port, PlaySound_2, sizeof(PlaySound_2)-1);
                printf("PlaySound_2\n"); ///
                clearPort(send_port);
                write(send_port, Neato_Testmode_On, sizeof(Neato_Testmode_On)-1);
                printf("Neato_Testmode_On\n");
            }
            if(button_state == 0)
            {
                value = 127;
                send_command(&flag, &value, send_port);//do left bumper released things
            }
            break;
        case BUTTON_RIGHT_BUMPER:
            flag = RIGHT_MOTOR_FLAG;
            if(button_state == 1)
            {
                value = 255;
                clearPort(send_port);
                write(send_port, PlaySound_8, sizeof(PlaySound_8)-1);
                printf("PlaySound_8\n"); ///
                clearPort(send_port);
                write(send_port, Neato_Testmode_Off, sizeof(Neato_Testmode_Off)-1);
                printf("Neato_Testmode_Off\n");
            }
            if(button_state == 0)
            {
                value = 127;
                send_command(&flag, &value, send_port);
            }
            break;
        case BUTTON_A:
            flag = SERVO_CLAW_RAISE_TAG;
            if(button_state == 1) {
                value = 0;
                printf("Lowering claw.\n");
                code_cmd = BUTTON_A;
                send_command(&flag, &value, send_port);//do A button pressed thing

                clearPort(send_port);
                write(send_port, PlaySound_1, sizeof(PlaySound_1)-1);
                printf("PlaySound_1\n");
                printf("NOT OK\n");
                lidar_to_use_flag = 0; // Lidar off
            }
            if(button_state == 0) {
                code_cmd = BUTTON_A + 10;
                send_command(&flag, &value, send_port);//do A button unpressed thing
            }
            break;
        case BUTTON_Y:
            flag = SERVO_CLAW_RAISE_TAG;
            if(button_state == 1) {
                value = 255;
                printf("Raising claw.\n");
                code_cmd = BUTTON_Y;
                send_command(&flag, &value, send_port);//do Y button pressed thing
                clearPort(send_port);
                write(send_port, PlaySound_9, sizeof(PlaySound_9)-1);
                printf("PlaySound_9\n");
                printf("OK\n");
                lidar_to_use_flag = 1; // Lidar on
            }
            if(button_state == 0) {
                code_cmd = BUTTON_Y + 10;
                send_command(&flag, &value, send_port);//do Y button unpressed thing
            }
            break;
        case BUTTON_X:
            flag = SERVO_CLAW_CLOSE_TAG;
            if(button_state == 1) {
                value = 255;
                printf("Closing claw.\n");
                code_cmd = BUTTON_X;
                send_command(&flag, &value, send_port); //do X button pressed thing
            }
            if(button_state == 0) {
                code_cmd = BUTTON_X+10;
                send_command(&flag, &value, send_port); //do X button unpressed thing
            }
            break;
        case BUTTON_B:
            flag = SERVO_CLAW_CLOSE_TAG;
            if(button_state == 1) {
                value = 0;
                printf("STOP &_Opening claw.\n");
                code_cmd = BUTTON_B;
                send_command(&flag, &value, send_port);//do B button pressed thing
            }
            if(button_state == 0) {
                code_cmd = BUTTON_B + 10;
                send_command(&flag, &value, send_port);//do B unbutton pressed thing
            }
            break;
        case BUTTON_BACK:
            flag = LEFT_MOTOR_STEPS_FLAG;
            if(button_state == 1)
            {
                int steps = 400;
                int seconds = 2;
                printf("\nMoving %i(%x) steps in %i(%x) seconds.\n", steps, steps, seconds, seconds);
                int n = write(send_port, &flag, 1);
                n = n + write(send_port, &steps, sizeof(steps));
                n = n + write(send_port, &seconds, sizeof(seconds));
                printf("Wrote %i bytes.\n", n);
            }
            break;
        case BUTTON_START:
            flag = RIGHT_MOTOR_STEPS_FLAG;
            if(button_state == 1)
            {
                int steps = 400;
                int seconds = 2;
                printf("\nMoving %i(%x) steps in %i(%x) seconds.\n", steps, steps, seconds, seconds);
                int n = write(send_port, &flag, 1);
                n = n + write(send_port, &steps, sizeof(steps));
                n = n + write(send_port, &seconds, sizeof(seconds));
                printf("Wrote %i bytes.\n", n);
            }
            break;
        case BUTTON_XBOX:
            flag = SENSOR_REQUEST;
            if ( button_state == 1 )
            {
                printf("Requesting sensor input\n");
                poll_sensors( send_port, receive_port );
            }
            break;
    }
    fflush(stdout);
    return 0;
}

int update_axis(int axis, int axis_value, int send_port)
{
    unsigned char flag = 0;
    unsigned char value = 0;
    switch(axis)
    {
        case AXIS_LEFT_STICK_VERTICAL:
            flag = LEFT_MOTOR_FLAG;
            value = map_stick(axis_value);
            if(value > 127 && (value - DEADZONE) < 127) //keeps stuff in the deadzone from being sent.
                value = 127;
            if(value < 127 && (value + DEADZONE) > 127)
                value = 127;
            printf("Driving left motor to speed: %i\n", value);
            send_command(&flag, &value, send_port);//do left stick up down thing
            break;
        case AXIS_RIGHT_STICK_VERTICAL:
            flag =RIGHT_MOTOR_FLAG;
            value = map_stick(axis_value);
            if(value > 127 && (value - DEADZONE) < 127) //keeps stuff in the deadzone from being sent.
                value = 127;
            if(value < 127 && (value + DEADZONE) > 127)
                value = 127;
            printf("Driving right motor to speed: %i\n", value);
            break;
    }

    return 0;
    fflush(stdout);
}

// timer_handler
void timer_handler(int sig)
{

    //printf("\ntimer speed %d path_left %d path_right %d  \n", path_speed, path_left, path_right);
    int cmd_lenght = 0;
    cmd_lenght = make_cmd_string(&n_drive_cmd, path_left, path_right, path_speed);
    clearPort(send_port);
    if (lidar_to_use_flag)   printf("YES_YES_YES\n");
    else  printf("NO_NO_NO\n");

    // no lidar
    if (lidar_to_use_flag ==0) {
        //printf("No_LIDAR\n");
            if (path_speed > 1) {
                write(send_port, n_drive_cmd, cmd_lenght);
                flag_neato_stop = 0;
                printf(n_drive_cmd);
                printf("\n");
            }
            if ((!flag_neato_stop) && (path_speed == 1)) {
                flag_neato_stop = 1;
                write(send_port, n_drive_cmd, cmd_lenght);
                printf("flag_neato_stop\n");
            }
    }
// -------
// Lidar present
//    if (lidar_to_use_flag >0) {
        //printf("LIDAR\n");
        if ((!Lidar_forward_stop_flag) || ((path_left < 0) && (path_right < 0))) {
            write(send_port, n_drive_cmd, cmd_lenght);
            printf(n_drive_cmd);
            printf("\n");
        } else if ((Lidar_forward_stop_flag) && (path_left > 0) && (path_right > 0)) {
            path_speed = 1;
            cmd_lenght = make_cmd_string(&n_drive_cmd, path_left, path_right, path_speed);
            write(send_port, n_drive_cmd, cmd_lenght);
            flag_neato_stop = 0;
        }
 //   }
// -----
    if ((!flag_neato_stop) && (path_speed == 1)) {
        flag_neato_stop = 1;
        write(send_port, n_drive_cmd, cmd_lenght);
    }

#ifdef DEBUG_JOYSTICK
        //printf("flag_neato_stop\n");
#endif
//    }
    g_called += 1;
    //printf("\n");

//    //printf("\ntimer speed %d path_left %d path_right %d  \n", path_speed, path_left, path_right);
//    int cmd_lenght = 0;
//    cmd_lenght = make_cmd_string(&n_drive_cmd, path_left, path_right, path_speed);
//    clearPort(send_port);
//    // no lidar
//    if (path_speed > 1) {
//        write(send_port, n_drive_cmd, cmd_lenght);
//        flag_neato_stop = 0;
//        printf(n_drive_cmd);
//        printf("\n");
//    }
//    if ((!flag_neato_stop) && (path_speed == 1)) {
//        flag_neato_stop = 1;
//        write(send_port, n_drive_cmd, cmd_lenght);
//        printf("flag_neato_stop\n");
//    }
//// -------
//// lidar present
//    //printf("TIMER\n");
//        if ((!Lidar_forward_stop_flag) || ((path_left < 0) && (path_right < 0)))  {
//            write(send_port, n_drive_cmd, cmd_lenght);
//            printf(n_drive_cmd);
//            printf("\n");
//        } else if ((Lidar_forward_stop_flag) && (path_left > 0) && (path_right >0)) {
//     //       printf("Lidar_Stop_CMD\n");
//            path_speed = 1;
//            cmd_lenght = make_cmd_string(&n_drive_cmd, path_left, path_right, path_speed);
//            write(send_port, n_drive_cmd, cmd_lenght);
//            flag_neato_stop = 0;
//        }
//    // -----
//
//    if ((!flag_neato_stop) && (path_speed == 1)) {
//        flag_neato_stop = 1;
//        write(send_port, n_drive_cmd, cmd_lenght);
//        #ifdef DEBUG_JOYSTICK
//		    printf("flag_neato_stop\n");
//	    #endif
//    }
//    g_called += 1;
//    //printf("\n");
}

//end of timer

// thread
int tty_fd;
unsigned char buf[1980];

typedef struct mymsgbuf {
    long mtype;
    unsigned char buf[1980];
    int num;
} mess_t;

typedef struct NEATOCMD{
    long mtype;
    char text[1];
    int path_speed;
    int path_left;
    int path_right;
    int lidar_flag;
} neato_t;

/* a little test program */
int main(int argc, char *argv[])
{
    //int  joy_file, received, send_port, receive_port; // file IDs for the joy, in serial, and out serial ports
    char buffer[512] = "";
    int  old_axis_values[8] = {0};     // intialize all buttons to "off" (0) this array is check edagainst for button updates and if an update is found the update is sent. Axis stuff is the same
    int  old_button_values[11] = {0};  // array as mentioned above
    int  new_button_values[11] = {0};  // just look above
    int  new_axis_values[8] = {0};     // just look above the above

    struct js_event jse;               // stores the joystick data

    if(argc > 2)
    {
        printf("Too many arguments, exiting."); // only one arg for the time being.
        exit(1);
    }
    if(argc == 2)
    {
        strcpy(joy_address, argv[1]);
        joy_file = open_joystick(joy_address); // sets the joystick to attach to, assumes that the arduino is mapped as seen in the ROBOT_DEFINITIONS.h
    }
    else if(argc == 1)
    {
        joy_file = open_joystick(joy_address); // if not specified use js0
    }

    if (joy_file < 0)
    {
        while(joy_file < 0)
        {
            printf("Joystick open failed, trying again in 1 sec.\n"); // a thing is broken and you should stop
            sleep(1);
            open_joystick(joy_address);
        }
    }

    send_port = serialport_init(VACUUM_COMM_LOCATION, ROBOT_BAUDRATE); // attempts to open the connection to the arduino with the BAUDRATE specified in the ROBOT_DEFINITIONS.h

    if(send_port < 0)
    {
        while(send_port < 0)
        {
            printf("Can't open vacuum serial port, trying again in 1 sec.\n"); // arduino not located, please stop breaking things
            sleep(1);
            send_port = serialport_init(VACUUM_COMM_LOCATION, ROBOT_BAUDRATE);
        }
    }
    clearPort(send_port);
    printf("send_port = %d\n", send_port );


#ifdef NEATO
    write(send_port, Neato_help, sizeof(Neato_help)-1);  // not work witout something to send
    write(send_port, Neato_Testmode_On, sizeof(Neato_Testmode_On)-1);
    printf("Neato_Testmode_On\n");
    clearPort(send_port);
    write(send_port, PlaySound_1, sizeof(PlaySound_1)-1);
    printf("PlaySound_1\n"); ///

    // timer
    struct sigaction act, oact;
    g_called = 0;

    sigemptyset(&act.sa_mask);
    act.sa_flags = 0;
    act.sa_handler = timer_handler;
    sigaction(SIGALRM, &act, &oact);

    struct itimerval tv;
    tv.it_value.tv_sec = 1; //time of first timer
    tv.it_value.tv_usec = 0; //time of first timer
    tv.it_interval.tv_sec = 0; //time of all timers but the first one
    tv.it_interval.tv_usec = 100000; //time of all timers but the first one
    //tv.it_interval.tv_usec = 50000; //time of all timers but the first one
    setitimer(ITIMER_REAL, &tv, NULL);

    sigset_t mask;
    sigprocmask(0, NULL, &mask);
    sigdelset(&mask, SIGALRM);
    //end of timer
#endif

    int counter = 0;


#ifdef LIDAR
    //const char default_port[] = "/dev/ttyUSB0";
    //const char default_port[] = "/dev/ttyACM3";
   // const char default_port[] = "/dev/ttyACM0";
    char *serial_port = (char *)lidar_port;
    if (2==argc) {
        serial_port = argv[1];
    }
    pid_t pid, joy_pid;
    pid_t neato_pid;

    int qid;
    int neato_qid;

    key_t msgkey;

    mess_t lidar_buf;
    neato_t neato_buf;

    int neato_l;
    int length;

    neato_l = sizeof(neato_t) - sizeof(long);
    length = sizeof(mess_t) - sizeof(long);
    msgkey = ftok(".", 'm');

    neato_qid = msgget(msgkey, IPC_CREAT | 0660);
    qid = msgget(msgkey, IPC_CREAT | 0660 );


    if(!(pid = fork())) {
        printf("Opening lidar serial port \n");
        tty_fd = open(serial_port, O_RDWR);
        if (tty_fd < 0) {
            printf("Could not open lidar port \n");
            return -1;
        }
        printf("lidar port is open \n");
        init_serial_port(tty_fd);

        srand(time (0));

        int ok = 0;
        printf("init_serial_port \n");
        while(1) {
            //printf("while(1) \n");
            if (1 == read(tty_fd, lidar_buf.buf, 1)){
                if(0xFA == lidar_buf.buf[0]) {
                    //printf("1 == read\n");
                    if (1 == read(tty_fd, lidar_buf.buf+1, 1))
                    {
                        if (0xA0 == lidar_buf.buf[1]) {
                            //printf("OK\n");
                            //printf(lidar_buf.buf[1]);
                            for (int idx = 2; idx < 1980; idx++) // register all the 360 readings (90 packets, 22 bytesh each)
                                read(tty_fd, lidar_buf.buf + idx, 1);

                            if (!count_errors(lidar_buf.buf)) { // if no errors during the transmission
                                lidar_buf.mtype = 1;
                                lidar_buf.num = rand() % 100;
                                int err = msgsnd(qid, &lidar_buf, length, 0);
                                //printf("SECOND FORK %d : %d  \n", lidar_buf.mtype, lidar_buf.num);
                                print_all_data(lidar_buf.buf);  // then print the data to the screen
                            }
                        }
                    }
                }
            }
        }
    }

#endif

    if(!(joy_pid = fork()))    {
        printf("Joystick_fork!\n");
        while(1){
            received = read_joystick_event(&jse); // check for a joystick update
            if (received == 1) { // if an update is available
            #ifdef DEBUG_JOYSTICK
                printf("receive: %hhu, %d, %d\n", jse.type, jse.value, jse.number);
            #endif
                switch (jse.type) {
                    case TYPE_BUTTON:
                        //printf("BUTTON\n");
                        button_update(&jse, new_button_values); // update teh new button array
                        break;
                    case TYPE_NOT_BUTTON:
                        //printf("NOT_BUTTON\n");
                        axis_update(&jse, new_axis_values); // udpate the new axis values array
                        break;
                }
            }
            counter++;
            if (counter == 10) {
                //printf("counter == 10\n");
                counter = 0;
                send_button_updates(old_button_values, new_button_values, send_port, receive_port); // checks bot the old and new button arrays for differences, if it finds one then an update is sent
                send_axis_updates(old_axis_values, new_axis_values, send_port); // ditto from above

                neato_buf.path_left = path_left;
                neato_buf.path_right = path_right;
                neato_buf.path_speed = path_speed;
                neato_buf.lidar_flag = lidar_to_use_flag;
                neato_buf.mtype = 1;
                //printf("fork joystic speed = %d, rigth = %d, left = %d\n", neato_buf.path_speed, neato_buf.path_right, neato_buf.path_left);
                msgsnd(neato_qid,&neato_buf,neato_l,0);
            }
            usleep(1000);
        }
    }

    /*if(!(neato_pid = fork()))    {
        while(1) {
            int neato_err = msgrcv(neato_qid, &neato_cmd, neato_l, 1, 0);
            if (neato_err != -1) {
                printf("fork neato speed = %d, rigth = %d, left = %d\n", neato_cmd.path_speed, neato_cmd.path_right,
                       neato_cmd.path_left);
                int cmd_lenght = 0;
                cmd_lenght = make_cmd_string(&n_drive_cmd, neato_cmd.path_left, neato_cmd.path_right, neato_cmd.path_speed);
                clearPort(send_port);
                // no lidar
                if (neato_cmd.path_speed > 1) {
                    write(send_port, n_drive_cmd, cmd_lenght);
                    flag_neato_stop = 0;
                    printf(n_drive_cmd);
                    printf("\n");
                }
                if ((!flag_neato_stop) && (neato_cmd.path_speed == 1)) {
                    flag_neato_stop = 1;
                    write(send_port, n_drive_cmd, cmd_lenght);
                    printf("flag_neato_stop\n");
                }

                if ((!flag_neato_stop) && (neato_cmd.path_speed == 1)) {
                    flag_neato_stop = 1;
                    write(send_port, n_drive_cmd, cmd_lenght);
#ifdef DEBUG_JOYSTICK
                    printf("flag_neato_stop\n");
#endif
                }
                g_called += 1;
            }
        }
    }*/

    while (1) {
        //print_all_data(buf.buf);
        int err_lidar;
        int err_joystyk;
        //------------------PACKET LIDAR---------------------------
        err_lidar = msgrcv(qid,&lidar_buf,length,1,0);
        //printf("%d\n", err);
        if (err_lidar != -1) {
            //printf("MAIN FORK %d : %d  \n", lidar_buf.mtype, lidar_buf.num);
            print_all_data(lidar_buf.buf);  // then print the data to the screen
        }
        //----------------------------------------------------------


        //-------------------PACKET JOYSTYK--------------------------
        err_joystyk = msgrcv(neato_qid,&neato_buf,neato_l,1,0);
        if(err_joystyk != -1)
        {
            lidar_to_use_flag = neato_buf.lidar_flag;
            path_left = neato_buf.path_left;
            path_right = neato_buf.path_right;
            path_speed = neato_buf.path_speed;
        }
        //--------------------------------------------------------------------------------------
    }

    printf("CLOSE\n");
    msgctl(qid, IPC_RMID, 0);

    #ifdef LIDAR
    close(tty_fd);
    #endif

}


