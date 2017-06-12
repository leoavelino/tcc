/**
 * @file test_controller.cpp
 *
 * Test Controller
 *
 * attitude_control -> transforma Ã¢ngulos em velocidades setpoints utilizando-se de um controlador P
 * rates_control -> transforma velocidades em torques utilizando-se de um controlador PD
 *
 * Controlador para teste criado com base no mc_att_control, porÃ©m sem utilizar quatÃ©rnios.
 *
 *
 * Created by Leonardo Avelino
 *
 **/

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>

/** MACROS    ******************/

/*  Ganhos Proporcionais de Ã¢ngulo roll, pitch e yaw*/
#define ROLL_P                 6.5f
#define PITCH_P                6.5f
#define YAW_P                  3.0f

/*  Ganhos Proporcionais de velocidade de roll, pitch e yaw*/
#define ROLL_RATES_P           0.15f
#define PITCH_RATES_P          0.15f
#define YAW_RATES_P            2.0f

/*  Ganhos Derivativos de velocidade de roll, pitch e yaw*/
#define ROLL_RATES_D           0.003f
#define PITCH_RATES_D          0.003f
#define YAW_RATES_D            0.0f


#define PI                      3.1415f

/********************************/

extern "C" __EXPORT int test_controller_main(int argc, char *argv[]);

struct v_data
{
    float roll;
    float pitch;
    float yaw;
};
typedef v_data v_data;


/* DefiniÃ§Ã£o da classe Test_Controller */
/* Classe que encapsula o controlador do tipo Test_Controller */
class Test_Controller{
public:

    /**
     * Construtor da classe
     */
    Test_Controller();

    /**
     * Destrutor, tambÃ©m mata a funÃ§Ã£o main()
     */
    ~Test_Controller();

    /**
     * FunÃ§Ã£o que inicia o Test_Controller
     * retorna OK caso tenha sido executada com sucesso
     */
    int start();

private:

    bool _task_should_exit;                                         // se for "true", task_main() termina;
    int _control_task;                                              // task handle

    int _v_attitude_sp_sub;                                         // vehicle attitude setpoint subscription
    int _v_attitude_sub;                                            // vehicle attitude subscription
    int _v_control_mode_sub;	                                    // vehicle control mode subscription
    int _armed_sub;                                                 // actuator armed subscription

    orb_advert_t	_actuators_0_pub;                           // actuator control publication

    struct vehicle_attitude_setpoint_s	_v_attitude_sp;             // vehicle attitude setpoint
    struct vehicle_attitude_s _attitude;                            // vehicle attitude
    struct vehicle_control_mode_s _v_control_mode;                  // vehicle control mode
    struct actuator_controls_s _actuators;			    // actuator controls
    struct actuator_armed_s _armed;                                 // arming status
    struct v_data _angles;                                          // current roll, pitch, yaw
    struct v_data _angles_sp;                                       // current angles setpoint
    struct v_data _rates;                                           // current rates
    struct v_data _rates_sp;                                        // current rates setpoint
    struct v_data _att_control;                                     // torques
    struct v_data _angles_p;                                        // ganhos P para os Ã¢ngulos
    struct v_data _rates_p, _rates_d;                               // ganhos P e D para as velocidades
    struct v_data _rates_prev_error;                                // erro anterior das velocidades, usado para calcular o termo derivativo

    perf_counter_t	_loop_perf;                                 /** loop performance counter */

    /**
     * Rotina chamada na inicializaÃ§Ã£o da thread.
     * Esta Ã© usada para chamar a funÃ§Ã£o task_main()
     */
    static void	task_main_trampoline(int argc, char *argv[]);

    /**
     * FunÃ§Ã£o que converte quaternios para Euler
     */
    void att_quaternion_to_euler(const float *q, struct v_data *angles);

    /**
     *  Checa se o veÃ­culo estÃ¡ armado
     */
    void arming_status_poll();

    /**
     * AquisiÃ§Ã£o de dados: ler dados de setpoint do veÃ­culo
     */
    void vehicle_attitude_setpoint_poll();

    /**
      * AquisiÃ§Ã£o de dados: ler dados de atitude atual/ velocidade atual do veÃ­culo (em roll, pitch, yaw)
      */
    void vehicle_attitude_poll();

    /**
      * AquisiÃ§Ã£o de dados: ler o estado de controle do veÃ­culo, vindos do commander. Usada neste cÃ³digo para ler flags de controle.
      * Os modos de vÃ´o setam diferentes flags. Por ex, o modo automÃ¡tico e o modo manual setam as flags de atitude e rates,
      * e estamos interessados nelas para fazer o controle.
      */
    void vehicle_control_mode_poll();

    /**
      * Controlador de atitude
      */
    void control_attitude();

    /**
      * Controlador de velocidade
      */
    void control_rates(float dt);
    /**
     * FunÃ§Ã£o principal
     */
    void task_main();
};

/* Escopo do cÃ³digo    */
namespace test_controller
{
    Test_Controller *g_control;
}

/*  Construtor da classe */
/*  InicializaÃ§Ã£o de variÃ¡veis e estrutras de dados */
Test_Controller::Test_Controller() :

   _task_should_exit(false),
   _control_task(-1),
   _v_attitude_sp_sub(-1),
   _v_attitude_sub(-1),
   _v_control_mode_sub(-1),
   _actuators_0_pub(nullptr),
   _v_attitude_sp{},
   _attitude{},
   _v_control_mode{},
   _actuators{},
   _armed{},
   _angles{},
   _angles_sp{},
   _rates{},
   _rates_sp{},
   _att_control{},
   _angles_p{},
   _rates_p{},
   _rates_d{},
   _rates_prev_error{},
   _loop_perf(perf_alloc(PC_ELAPSED, "test_controller"))
{}

/*   Destrutor da classe   */
/*  Desaloca dados e encerra a funÃ§Ã£o principal */
Test_Controller::~Test_Controller()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    test_controller::g_control = nullptr;
}

void Test_Controller::arming_status_poll()
{
    /* checa se existe algum dado novo */
    bool updated;
    orb_check(_armed_sub, &updated);

    /*caso positivo, copia dados novos*/
    if (updated) {
        orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
    }
}

void Test_Controller::vehicle_attitude_setpoint_poll()
{
    /* checa se existe algum dado novo */
    bool updated;
    orb_check(_v_attitude_sp_sub, &updated);

    /*caso positivo, copia dados novos*/
    if(updated) {
        orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_attitude_sp_sub, &_v_attitude_sp);
    }

    /*Copiar os dados de atitude setpoint para a estrutura que serÃ¡ utilizada pelo controlador*/
    _angles_sp.roll = _v_attitude_sp.roll_body;
    _angles_sp.pitch = _v_attitude_sp.pitch_body;
    _angles_sp.yaw = _v_attitude_sp.yaw_body;
}

void Test_Controller::vehicle_attitude_poll()
{
    /* checa se existe algum dado novo */
    bool updated;
    orb_check(_v_attitude_sub, &updated);

    /*caso positivo, copia dados novos*/
    if(updated) {
        orb_copy(ORB_ID(vehicle_attitude), _v_attitude_sub, &_attitude);
    }
    /*Converter os dados adquiridos para Euler*/
    att_quaternion_to_euler(_attitude.q, &_angles);

    /*Copiar os dados de velocidade para a estrutura que serÃ¡ utilizado pelo controlador*/
    _rates.roll = _attitude.rollspeed;
    _rates.pitch = _attitude.pitchspeed;
    _rates.yaw = _attitude.yawspeed;
}

void Test_Controller::vehicle_control_mode_poll()
{
    bool updated;

    /* checa se existe algum dado novo */
    orb_check(_v_control_mode_sub, &updated);

    /*caso positivo, copia dados novos*/
    if (updated) {
        orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
    }
}

/*Simplesmente recebe um quatÃ©rnio e uma estrutura do tipo v_data por referÃªncia, converte o quatÃ©rnio para euler */
/*  e grava na estrutura                            */
void Test_Controller::att_quaternion_to_euler(const float *q, struct v_data *angles)
{
    matrix::Eulerf att_euler = matrix::Quatf(q);
    angles->roll = att_euler(0);
    angles->pitch = att_euler(1);
    angles->yaw = att_euler(2);
}

/*Controlador de Ã‚ngulos*/
/*Entrada: angulos atuais, angulos setpoints   */
/*Saida: velocidades setpoint   */

void Test_Controller::control_attitude()
{
    /* Checa se existe um novo setpoint de Ã¢ngulo*/
    vehicle_attitude_setpoint_poll();
    /* checa se o veÃ­culo mudou de posiÃ§Ã£o*/
    vehicle_attitude_poll();

    /* calcular error */
    struct v_data angles_error;
    angles_error.roll = _angles_sp.roll - _angles.roll;
    angles_error.pitch = _angles_sp.pitch - _angles.pitch;
    /* Verifica se Ã© o menor caminho de yaw com o uso da funÃ§Ã£o _wrap_pi*/
    angles_error.yaw = _wrap_pi(_angles_sp.yaw - _angles.yaw);

    /** Proportional control */
    _angles_p.roll = ROLL_P * angles_error.roll;
    _angles_p.pitch = PITCH_P * angles_error.pitch;
    _angles_p.yaw = YAW_P * angles_error.yaw;

    /** Velocidades setpoints calculadas */
    _rates_sp.roll = _angles_p.roll;
    _rates_sp.pitch = _angles_p.pitch;
    _rates_sp.yaw = _angles_p.yaw;
}

/*Controlador de Velocidade*/
/*Entrada: velocidades atuais, Velocidades setpoints   */
/*Saida: Torques   */

void Test_Controller::control_rates(float dt)
{
    /* checa se o veÃ­culo mudou de posiÃ§Ã£o*/
    vehicle_attitude_poll();

    /* calcular error */
    struct v_data rates_error;
    rates_error.roll = _rates_sp.roll - _rates.roll;
    rates_error.pitch = _rates_sp.pitch - _rates.pitch;
    rates_error.yaw = _rates_sp.yaw - _rates.yaw;


    /** Proportional control */
    _rates_p.roll = ROLL_RATES_P * rates_error.roll;
    _rates_p.pitch = PITCH_RATES_P * rates_error.pitch;
    _rates_p.yaw = YAW_RATES_P * rates_error.yaw;

    /** Derivative control */
    _rates_d.roll = ((rates_error.roll - _rates_prev_error.roll) / dt) * ROLL_RATES_D;
    _rates_d.pitch = ((rates_error.pitch - _rates_prev_error.pitch) / dt) * PITCH_RATES_D;
    _rates_d.yaw = ((rates_error.yaw - _rates_prev_error.yaw) / dt) * YAW_RATES_D;

    /* Salva o erro atual para prÃ³xima iteraÃ§Ã£o */
    _rates_prev_error.roll = rates_error.roll;
    _rates_prev_error.pitch = rates_error.pitch;
    _rates_prev_error.yaw = rates_error.yaw;

    /* Calcular os torques */
    _att_control.roll = _rates_p.roll + _rates_d.roll;
    _att_control.pitch = _rates_p.pitch + _rates_d.pitch;
    _att_control.yaw = _rates_p.yaw + _rates_d.yaw;

}

/* FunÃ§Ã£o que cria um thread para este mÃ³dulo*/
int Test_Controller::start()
{
    ASSERT(_control_task == -1);
    /* start the task */
    _control_task = px4_task_spawn_cmd("test_controller",                               //nome da thread
                                       SCHED_DEFAULT,                                   //escalonador que serÃ¡ utilizado
                                       SCHED_PRIORITY_MAX,                              //prioridade da thread
                                       2048,                                            //stack utilizada pela thread
                                       (px4_main_t)&task_main_trampoline,               //rotina que serÃ¡ chamada quando a thread for criada
                                       nullptr);
    if (_control_task < 0){
        warn("Task start failed");
        return -errno;
    }
    return OK;
}


void Test_Controller::task_main_trampoline(int argc, char *argv[])
{
   /* O que serÃ¡ feito quando a thread for criada estarÃ¡ nessa rotina*/

   /*Neste caso, passa-se o controle para a funÃ§Ã£o task_main() */
   test_controller::g_control->task_main();
}

void Test_Controller::task_main()
{
    /* Inscrever nos tÃ³picos */
    _v_attitude_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    _v_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _armed_sub = orb_subscribe(ORB_ID(actuator_armed));

    /*Criar a estrutura que iremos fazer polling... Esta funÃ§Ã£o Ã© bem interessante, e nos permite colocar a thread para dormir, atÃ© */
    /* que dados do tÃ³pico que estamos inspecionando chegue. A vantagem Ã© que a CPU pode executar outras threads enquanto nÃ£o recebe-se */
    /*    os dados desejados */
    px4_pollfd_struct_t poll_fds = {};
    poll_fds.events = POLLIN;

    /*Loop principal. SerÃ¡ executado sempre atÃ© que um sinal para matar a thread seja enviado*/
    while(!_task_should_exit)
    {
        /*TÃ³pico que iremos fazer polling: vehicle_attitude*/
        poll_fds.fd = _v_attitude_sub;

        // Bloqueie a thread e espere por atÃ© 100 ms por dados do tÃ³pico vehicle_attitude
        int pret = px4_poll(&poll_fds, 1, 100);

        // tempo expirado
        if (pret == 0) {
            continue;
        }

        /* caso algum erro retorne da funÃ§Ã£o POLL */
        if (pret < 0) {
            warn("pid att ctrl: poll error %d, %d", pret, errno);
            /* sleep a bit before next try */
            usleep(100000);
            continue;
        }
        /* Contador para checar em quanto tempo o loop de controle estÃ¡ sendo executado. A vantagem Ã© que, por ser implementado por um driver,
         * ele nÃ£o Ã© bloqueado por outras aplicaÃ§Ãµes.*/
        /* Ele comeÃ§a aqui*/
        perf_begin(_loop_perf);

        /*Se existir novos dados do tÃ³pico que estÃ¡ sendo investigado, entre aqui*/
        if (poll_fds.revents & POLLIN) {

            /*obtenha o tempo atual*/
            static uint64_t last_run = 0;
            float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
            last_run = hrt_absolute_time();

            /* guarde contra dt's muito pequeno (< 2ms) e muito grande (> 20ms)*/
            if (dt < 0.002f) {
                dt = 0.002f;
            }
            else if (dt > 0.02f) {
                dt = 0.02f;
            }

            /*Copie os dados obtidos do tÃ³pico inspecionado*/
            orb_copy(ORB_ID(vehicle_attitude), _v_attitude_sub, &_attitude);

            /* Verifique se existem novos dados com as rotinas abaixo*/
            arming_status_poll();
            vehicle_control_mode_poll();

            /*Se a flag de controle de atitude (Ã¢ngulos) estiver habilitada, execute controle de atitude*/
            if (_v_control_mode.flag_control_attitude_enabled)
            {
                control_attitude();
            }

            /*Se a flag de controle de velocidade estiver habilitada, execute controle de velocidade e publique torques*/
            if (_v_control_mode.flag_control_rates_enabled)
            {
                control_rates(dt);

                /*Como nÃ£o estamos controlando altura, ler o valor de throttle publicado pelo mc_pos_control e use-o*/
                float thrust_sp = _v_attitude_sp.thrust;

                /*Copie os torques calculados, mas antes cheque se os valores convergiram */
                _actuators.control[0] = (PX4_ISFINITE(_att_control.roll)) ? _att_control.roll : 0.0f;   //torque em roll
                _actuators.control[1] = (PX4_ISFINITE(_att_control.pitch)) ? _att_control.pitch : 0.0f; //torque em pitch
                _actuators.control[2] = (PX4_ISFINITE(_att_control.yaw)) ? _att_control.yaw : 0.0f;     //torque em yaw
                _actuators.control[3] = (PX4_ISFINITE(thrust_sp)) ? thrust_sp : 0.0f;                   //throttle

                /*Os torques sÃ£o publicados aqui*/
                if (_actuators_0_pub != nullptr) {

                    orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators);
                }
                else
                {
                    _actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators);
                }
            }
        }
        perf_end(_loop_perf); //o contador do loop de controle encerra aqui
    } _control_task = -1;
}

/* Rotina main() (principal) do mÃ³dulo */
/* Esta rotina Ã© iniciada quando o mÃ³dulo Ã© iniciado, ela Ã© responsÃ¡vel por instanciar um objeto da classe Test_Controller*/
/* ou destruir um.   */

int test_controller_main(int argc, char* argv[])
{
    /* envia um warning se os argumentos forem invÃ¡lidos */
    if (argc < 2){
        warnx("usage: test_controller {start|stop|status}");
        return 1;
    }
    /* iniciar a thread test_controller manualmente */
    if (!strcmp(argv[1], "start")) {
        if (test_controller::g_control != nullptr) {
            warnx("Already running!");
            return 1;
        }
        test_controller::g_control = new Test_Controller;
        if (test_controller::g_control == nullptr) {
            warnx("Allocation failed.");
            return 1;
        }
        if (OK != test_controller::g_control->start()) {
            delete test_controller::g_control;
            test_controller::g_control = nullptr;
            warnx("Start failed.");
            return 1;
        }
        return 0;
    }
    /* parar a thread test_controller manualmente */
    if (!strcmp(argv[1], "stop")) {
        if (test_controller::g_control == nullptr) {
            warnx("Not running.");
            return 1;
        }
        delete test_controller::g_control;
        test_controller::g_control = nullptr;
        return 0;
    }
    /* Retorna o status da aplicaÃ§Ã£o */
    if (!strcmp(argv[1], "status")) {
        if (test_controller::g_control) {
            warnx("Running.");
            return 0;
        } else {
            warnx("Not running.");
            return 1;
        }
    }

    /* Se nenhum argumento citado for utilizado */
    warnx("Unrecognized command.");
    return 0;
}
