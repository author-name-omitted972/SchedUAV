#include "ThreadCtl.hpp"

#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <vector>
#include <tuple>
#include <iostream>
#include <fstream>
#include <sstream>

int main(void)
{
    // lio_omp_0
    ThreadCtl lio_omp_0("lio_omp_0");
    lio_omp_0.set_policy(SCHED_OTHER, 0);
    printf("lio_omp_0 tgid: %d, tid: %d, policy: %d, priority: %d\n", lio_omp_0.pid(), lio_omp_0.tid(), lio_omp_0.policy(), lio_omp_0.priority());

    // lio_omp_1
    ThreadCtl lio_omp_1("lio_omp_1");
    lio_omp_1.set_policy(SCHED_OTHER, 0);
    printf("lio_omp_1 tgid: %d, tid: %d, policy: %d, priority: %d\n", lio_omp_1.pid(), lio_omp_1.tid(), lio_omp_1.policy(), lio_omp_1.priority());

    // lio_omp_2
    ThreadCtl lio_omp_2("lio_omp_2");
    lio_omp_2.set_policy(SCHED_OTHER, 0);
    printf("lio_omp_2 tgid: %d, tid: %d, policy: %d, priority: %d\n", lio_omp_2.pid(), lio_omp_2.tid(), lio_omp_2.policy(), lio_omp_2.priority());

    // lio_ikd
    ThreadCtl lio_ikd("lio_ikd");
    lio_ikd.set_policy(SCHED_OTHER, 0);
    printf("lio_ikd tgid: %d, tid: %d, policy: %d, priority: %d\n", lio_ikd.pid(), lio_ikd.tid(), lio_ikd.policy(), lio_ikd.priority());

    // ego_waypoint
    ThreadCtl ego_waypoint("ego_waypoint");
    ego_waypoint.set_policy(SCHED_OTHER, 0);
    printf("ego_waypoint tgid: %d, tid: %d, policy: %d, priority: %d\n", ego_waypoint.pid(), ego_waypoint.tid(), ego_waypoint.policy(), ego_waypoint.priority());

    // ego_odometry
    ThreadCtl ego_odometry("ego_odometry");
    ego_odometry.set_policy(SCHED_OTHER, 0);
    printf("ego_odometry tgid: %d, tid: %d, policy: %d, priority: %d\n", ego_odometry.pid(), ego_odometry.tid(), ego_odometry.policy(), ego_odometry.priority());

    // ego_execFSM
    ThreadCtl ego_execFSM("ego_execFSM");
    ego_execFSM.set_policy(SCHED_OTHER, 0);
    printf("ego_execFSM tgid: %d, tid: %d, policy: %d, priority: %d\n", ego_execFSM.pid(), ego_execFSM.tid(), ego_execFSM.policy(), ego_execFSM.priority());
    
    // ego_checkColl
    ThreadCtl ego_checkColl("ego_checkColl");
    ego_checkColl.set_policy(SCHED_OTHER, 0);
    printf("ego_checkColl tgid: %d, tid: %d, policy: %d, priority: %d\n", ego_checkColl.pid(), ego_checkColl.tid(), ego_checkColl.policy(), ego_checkColl.priority());
    
    // ego_vis
    ThreadCtl ego_vis("ego_vis");
    ego_vis.set_policy(SCHED_OTHER, 0);
    printf("ego_vis tgid: %d, tid: %d, policy: %d, priority: %d\n", ego_vis.pid(), ego_vis.tid(), ego_vis.policy(), ego_vis.priority());

    // ego_updateOccu
    ThreadCtl ego_updateOcc("ego_updateOcc");
    ego_updateOcc.set_policy(SCHED_OTHER, 0);
    printf("ego_updateOcc tgid: %d, tid: %d, policy: %d, priority: %d\n", ego_updateOcc.pid(), ego_updateOcc.tid(), ego_updateOcc.policy(), ego_updateOcc.priority());

    // ego_depthOdom
    ThreadCtl ego_depthOdom("ego_depthOdom");
    ego_depthOdom.set_policy(SCHED_OTHER, 0);
    printf("ego_depthOdom tgid: %d, tid: %d, policy: %d, priority : %d\n", ego_depthOdom.pid(), ego_depthOdom.tid(), ego_depthOdom.policy(), ego_depthOdom.priority());
    
    // traj_main
    ThreadCtl traj_main("traj_main");
    traj_main.set_policy(SCHED_OTHER, 0);
    printf("traj_main tgid: %d, tid: %d, policy: %d, priority: %d\n", traj_main.pid(), traj_main.tid(), traj_main.policy(), traj_main.priority());

    // offboard_main
    ThreadCtl offboard_main("offboard_main");
    offboard_main.set_policy(SCHED_OTHER, 0);
    printf("offboard_main tgid: %d, tid: %d, policy: %d, priority: %d\n", offboard_main.pid(), offboard_main.tid(), offboard_main.policy(), offboard_main.priority());

    // wq:vehicle_imu
    ThreadCtl vehicle_imu("wq:vehicle_imu");
    vehicle_imu.set_policy(SCHED_OTHER, 0);
    printf("vehicle_imu tgid: %d, tid: %d, policy: %d, priority: %d\n", vehicle_imu.pid(), vehicle_imu.tid(), vehicle_imu.policy(), vehicle_imu.priority());

    // wq:vehicle_rate
    ThreadCtl vehicle_rate("wq:vehicle_rate");
    vehicle_rate.set_policy(SCHED_OTHER, 0);
    printf("vehicle_rate tgid: %d, tid: %d, policy: %d, priority: %d\n", vehicle_rate.pid(), vehicle_rate.tid(), vehicle_rate.policy(), vehicle_rate.priority());

    // wq:vehicle_acc
    ThreadCtl vehicle_acc("wq:vehicle_acc");
    vehicle_acc.set_policy(SCHED_OTHER, 0);
    printf("vehicle_acc tgid: %d, tid: %d, policy: %d, priority: %d\n", vehicle_acc.pid(), vehicle_acc.tid(), vehicle_acc.policy(), vehicle_acc.priority());

    // wq:vehicle_mag
    ThreadCtl vehicle_mag("wq:vehicle_mag");
    vehicle_mag.set_policy(SCHED_OTHER, 0);
    printf("vehicle_mag tgid: %d, tid: %d, policy: %d, priority: %d\n", vehicle_mag.pid(), vehicle_mag.tid(), vehicle_mag.policy(), vehicle_mag.priority());

    // wq:vehicle_air
    ThreadCtl vehicle_air("wq:vehicle_air");
    vehicle_air.set_policy(SCHED_OTHER, 0);
    printf("vehicle_air tgid: %d, tid: %d, policy: %d, priority: %d\n", vehicle_air.pid(), vehicle_air.tid(), vehicle_air.policy(), vehicle_air.priority());

    // wq:sensors
    ThreadCtl sensors("wq:sensors");
    sensors.set_policy(SCHED_OTHER, 0);
    printf("sensors tgid: %d, tid: %d, policy: %d, priority: %d\n", sensors.pid(), sensors.tid(), sensors.policy(), sensors.priority());

    // wq:ekf2
    ThreadCtl ekf2("wq:ekf2");
    ekf2.set_policy(SCHED_OTHER, 0);
    printf("ekf2 tgid: %d, tid: %d, policy: %d, priority: %d\n", ekf2.pid(), ekf2.tid(), ekf2.policy(), ekf2.priority());

    // wq:mc_hover_est
    ThreadCtl mc_hover_est("wq:mc_hover_est");
    mc_hover_est.set_policy(SCHED_OTHER, 0);
    printf("mc_hover_est tgid: %d, tid: %d, policy: %d, priority: %d\n", mc_hover_est.pid(), mc_hover_est.tid(), mc_hover_est.policy(), mc_hover_est.priority());

    // wq:land_det
    ThreadCtl land_det("wq:land_det");
    land_det.set_policy(SCHED_OTHER, 0);
    printf("land_det tgid: %d, tid: %d, policy: %d, priority: %d\n", land_det.pid(), land_det.tid(), land_det.policy(), land_det.priority());

    // wq:mc_rate_ctl
    ThreadCtl mc_rate_ctl("wq:mc_rate_ctl");
    mc_rate_ctl.set_policy(SCHED_OTHER, 0);
    printf("mc_rate_ctl tgid: %d, tid: %d, policy: %d, priority: %d\n", mc_rate_ctl.pid(), mc_rate_ctl.tid(), mc_rate_ctl.policy(), mc_rate_ctl.priority());

    // wq:mc_att_ctl
    ThreadCtl mc_att_ctl("wq:mc_att_ctl");
    mc_att_ctl.set_policy(SCHED_OTHER, 0);
    printf("mc_att_ctl tgid: %d, tid: %d, policy: %d, priority: %d\n", mc_att_ctl.pid(), mc_att_ctl.tid(), mc_att_ctl.policy(), mc_att_ctl.priority());

    // wq:mc_pos_ctl
    ThreadCtl mc_pos_ctl("wq:mc_pos_ctl");
    mc_pos_ctl.set_policy(SCHED_OTHER, 0);
    printf("mc_pos_ctl tgid: %d, tid: %d, policy: %d, priority: %d\n", mc_pos_ctl.pid(), mc_pos_ctl.tid(), mc_pos_ctl.policy(), mc_pos_ctl.priority());

    // wq:flt_mod_man
    ThreadCtl flt_mod_man("wq:flt_mod_man");
    flt_mod_man.set_policy(SCHED_OTHER, 0);
    printf("flt_mod_man tgid: %d, tid: %d, policy: %d, priority: %d\n", flt_mod_man.pid(), flt_mod_man.tid(), flt_mod_man.policy(), flt_mod_man.priority());
  
    // wq:ctl_alloc
    ThreadCtl ctl_alloc("wq:ctl_alloc");
    ctl_alloc.set_policy(SCHED_OTHER, 0);
    printf("ctl_alloc tgid: %d, tid: %d, policy: %d, priority: %d\n", ctl_alloc.pid(), ctl_alloc.tid(), ctl_alloc.policy(), ctl_alloc.priority());

    // wq:pwm_out_sim
    ThreadCtl pwm_out_sim("wq:pwm_out_sim");
    pwm_out_sim.set_policy(SCHED_OTHER, 0);
    printf("pwm_out_sim tgid: %d, tid: %d, policy: %d, priority: %d\n", pwm_out_sim.pid(), pwm_out_sim.tid(), pwm_out_sim.policy(), pwm_out_sim.priority());

    return 0;
}