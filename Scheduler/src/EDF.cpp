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
    lio_omp_0.set_policy(SCHED_DEADLINE, 0, 50*1000*1000, 50*1000*1000, 50*1000*1000);
    printf("lio_omp_0 tgid: %d, tid: %d, policy: %d\n", lio_omp_0.pid(), lio_omp_0.tid(), lio_omp_0.policy());

    // lio_omp_1
    ThreadCtl lio_omp_1("lio_omp_1");
    lio_omp_1.set_policy(SCHED_DEADLINE, 0, 50*1000*1000, 50*1000*1000, 50*1000*1000);
    printf("lio_omp_1 tgid: %d, tid: %d, policy: %d\n", lio_omp_1.pid(), lio_omp_1.tid(), lio_omp_1.policy());

    // lio_omp_2
    ThreadCtl lio_omp_2("lio_omp_2");
    lio_omp_2.set_policy(SCHED_DEADLINE, 0, 50*1000*1000, 50*1000*1000, 50*1000*1000);
    printf("lio_omp_2 tgid: %d, tid: %d, policy: %d\n", lio_omp_2.pid(), lio_omp_2.tid(), lio_omp_2.policy());

    // lio_ikd
    ThreadCtl lio_ikd("lio_ikd");
    lio_ikd.set_policy(SCHED_DEADLINE, 0, 5*1000*1000, 5*1000*1000, 5*1000*1000);
    printf("lio_ikd tgid: %d, tid: %d, policy: %d\n", lio_ikd.pid(), lio_ikd.tid(), lio_ikd.policy());

    // ego_waypoint
    ThreadCtl ego_waypoint("ego_waypoint");
    ego_waypoint.set_policy(SCHED_DEADLINE, 0, 50*1000*1000, 50*1000*1000, 50*1000*1000);
    printf("ego_waypoint tgid: %d, tid: %d, policy: %d\n", ego_waypoint.pid(), ego_waypoint.tid(), ego_waypoint.policy());

    // ego_odometry
    ThreadCtl ego_odometry("ego_odometry");
    ego_odometry.set_policy(SCHED_DEADLINE, 0, 50*1000*1000, 50*1000*1000, 50*1000*1000);
    printf("ego_odometry tgid: %d, tid: %d, policy: %d\n", ego_odometry.pid(), ego_odometry.tid(), ego_odometry.policy());

    // ego_execFSM
    ThreadCtl ego_execFSM("ego_execFSM");
    ego_execFSM.set_policy(SCHED_DEADLINE, 0, 10*1000*1000, 10*1000*1000, 10*1000*1000);
    printf("ego_execFSM tgid: %d, tid: %d, policy: %d\n", ego_execFSM.pid(), ego_execFSM.tid(), ego_execFSM.policy());

    // ego_checkColl
    ThreadCtl ego_checkColl("ego_checkColl");
    ego_checkColl.set_policy(SCHED_DEADLINE, 0, 50*1000*1000, 50*1000*1000, 50*1000*1000);
    printf("ego_checkColl tgid: %d, tid: %d, policy: %d\n", ego_checkColl.pid(), ego_checkColl.tid(), ego_checkColl.policy());

    // ego_vis
    ThreadCtl ego_vis("ego_vis");
    ego_vis.set_policy(SCHED_DEADLINE, 0, 110*1000*1000, 110*1000*1000, 110*1000*1000);
    printf("ego_vis tgid: %d, tid: %d, policy: %d\n", ego_vis.pid(), ego_vis.tid(), ego_vis.policy());

    // ego_updateOcc
    ThreadCtl ego_updateOcc("ego_updateOcc");
    ego_updateOcc.set_policy(SCHED_DEADLINE, 0, 50*1000*1000, 50*1000*1000, 50*1000*1000);
    printf("ego_updateOcc tgid: %d, tid: %d, policy: %d\n", ego_updateOcc.pid(), ego_updateOcc.tid(), ego_updateOcc.policy());

    // ego_depthOdom
    ThreadCtl ego_depthOdom("ego_depthOdom");
    ego_depthOdom.set_policy(SCHED_DEADLINE, 0, 50*1000*1000, 50*1000*1000, 50*1000*1000);
    printf("ego_depthOdom tgid: %d, tid: %d, policy: %d\n", ego_depthOdom.pid(), ego_depthOdom.tid(), ego_depthOdom.policy());

    // traj_main
    ThreadCtl traj_main("traj_main");
    traj_main.set_policy(SCHED_DEADLINE, 0, 10*1000*1000, 10*1000*1000, 10*1000*1000);
    printf("traj_main tgid: %d, tid: %d, policy: %d\n", traj_main.pid(), traj_main.tid(), traj_main.policy());

    // offboard_main
    ThreadCtl offboard_main("offboard_main");
    offboard_main.set_policy(SCHED_DEADLINE, 0, 20*1000*1000, 20*1000*1000, 20*1000*1000);
    printf("offboard_main tgid: %d, tid: %d, policy: %d\n", offboard_main.pid(), offboard_main.tid(), offboard_main.policy());
    
    // wq:vehicle_imu
    ThreadCtl wq_vehicle_imu("wq:vehicle_imu");
    wq_vehicle_imu.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:vehicle_imu tgid: %d, tid: %d, policy: %d\n", wq_vehicle_imu.pid(), wq_vehicle_imu.tid(), wq_vehicle_imu.policy());

    // wq:vehicle_rate
    ThreadCtl wq_vehicle_rate("wq:vehicle_rate");
    wq_vehicle_rate.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:vehicle_rate tgid: %d, tid: %d, policy: %d\n", wq_vehicle_rate.pid(), wq_vehicle_rate.tid(), wq_vehicle_rate.policy());

    // wq:vehicle_acc
    ThreadCtl wq_vehicle_acc("wq:vehicle_acc");
    wq_vehicle_acc.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:vehicle_acc tgid: %d, tid: %d, policy: %d\n", wq_vehicle_acc.pid(), wq_vehicle_acc.tid(), wq_vehicle_acc.policy());

    // wq:vehicle_mag
    ThreadCtl wq_vehicle_mag("wq:vehicle_mag");
    wq_vehicle_mag.set_policy(SCHED_DEADLINE, 0, 33*1000*1000, 33*1000*1000, 33*1000*1000);
    printf("wq:vehicle_mag tgid: %d, tid: %d, policy: %d\n", wq_vehicle_mag.pid(), wq_vehicle_mag.tid(), wq_vehicle_mag.policy());

    // wq:vehicle_air
    ThreadCtl wq_vehicle_air("wq:vehicle_air");
    wq_vehicle_air.set_policy(SCHED_DEADLINE, 0, 33*1000*1000, 33*1000*1000, 33*1000*1000);
    printf("wq:vehicle_air tgid: %d, tid: %d, policy: %d\n", wq_vehicle_air.pid(), wq_vehicle_air.tid(), wq_vehicle_air.policy());

    // wq:sensors
    ThreadCtl wq_sensors("wq:sensors");
    wq_sensors.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:sensors tgid: %d, tid: %d, policy: %d\n", wq_sensors.pid(), wq_sensors.tid(), wq_sensors.policy());

    // wq:ekf2
    ThreadCtl wq_ekf2("wq:ekf2");
    wq_ekf2.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:ekf2 tgid: %d, tid: %d, policy: %d\n", wq_ekf2.pid(), wq_ekf2.tid(), wq_ekf2.policy());

    // wq:mc_hover_est
    ThreadCtl wq_mc_hover_est("wq:mc_hover_est");
    wq_mc_hover_est.set_policy(SCHED_DEADLINE, 0, 8*1000*1000, 8*1000*1000, 8*1000*1000);
    printf("wq:mc_hover_est tgid: %d, tid: %d, policy: %d\n", wq_mc_hover_est.pid(), wq_mc_hover_est.tid(), wq_mc_hover_est.policy());

    // wq:land_det
    ThreadCtl wq_land_det("wq:land_det");
    wq_land_det.set_policy(SCHED_DEADLINE, 0, 8*1000*1000, 8*1000*1000, 8*1000*1000);
    printf("wq:land_det tgid: %d, tid: %d, policy: %d\n", wq_land_det.pid(), wq_land_det.tid(), wq_land_det.policy());

    // wq:mc_rate_ctl
    ThreadCtl wq_mc_rate_ctl("wq:mc_rate_ctl");
    wq_mc_rate_ctl.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:mc_rate_ctl tgid: %d, tid: %d, policy: %d\n", wq_mc_rate_ctl.pid(), wq_mc_rate_ctl.tid(), wq_mc_rate_ctl.policy());

    // wq:mc_att_ctl
    ThreadCtl wq_mc_att_ctl("wq:mc_att_ctl");
    wq_mc_att_ctl.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:mc_att_ctl tgid: %d, tid: %d, policy: %d\n", wq_mc_att_ctl.pid(), wq_mc_att_ctl.tid(), wq_mc_att_ctl.policy());

    // wq:mc_pos_ctl
    ThreadCtl wq_mc_pos_ctl("wq:mc_pos_ctl");
    wq_mc_pos_ctl.set_policy(SCHED_DEADLINE, 0, 8*1000*1000, 8*1000*1000, 8*1000*1000);
    printf("wq:mc_pos_ctl tgid: %d, tid: %d, policy: %d\n", wq_mc_pos_ctl.pid(), wq_mc_pos_ctl.tid(), wq_mc_pos_ctl.policy());

    // wq:flt_mod_man
    ThreadCtl wq_flt_mod_man("wq:flt_mod_man");
    wq_flt_mod_man.set_policy(SCHED_DEADLINE, 0, 20*1000*1000, 20*1000*1000, 20*1000*1000);
    printf("wq:flt_mod_man tgid: %d, tid: %d, policy: %d\n", wq_flt_mod_man.pid(), wq_flt_mod_man.tid(), wq_flt_mod_man.policy());

    // wq:ctl_alloc 
    ThreadCtl wq_ctl_alloc("wq:ctl_alloc");
    wq_ctl_alloc.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:ctl_alloc tgid: %d, tid: %d, policy: %d\n", wq_ctl_alloc.pid(), wq_ctl_alloc.tid(), wq_ctl_alloc.policy());

    // wq:pwm_out_sim
    ThreadCtl wq_pwm_out_sim("wq:pwm_out_sim");
    wq_pwm_out_sim.set_policy(SCHED_DEADLINE, 0, 4*1000*1000, 4*1000*1000, 4*1000*1000);
    printf("wq:pwm_out_sim tgid: %d, tid: %d, policy: %d\n", wq_pwm_out_sim.pid(), wq_pwm_out_sim.tid(), wq_pwm_out_sim.policy());

    return 0;
}
