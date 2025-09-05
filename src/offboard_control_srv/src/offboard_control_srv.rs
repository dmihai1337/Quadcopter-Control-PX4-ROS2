use rclrs::*;
use std::{thread, time::Duration};
use std::sync::{Arc, Mutex};
use px4_msgs::msg::{
    VehicleCommand as VehicleCommandMsg, 
    VehicleCommandAck, 
    VehicleLocalPosition, 
    VehicleLandDetected, 
    OffboardControlMode, 
    TrajectorySetpoint
};
use px4_msgs::srv::{
    VehicleCommand as VehicleCommandSrv,
    VehicleCommand_Request, 
    VehicleCommand_Response
};

pub struct OffboardControlNode {

    node: Node,

    // ======================================= CLIENT  ======================================= //

    vehicle_command_client: Client<VehicleCommandSrv>,

    // ======================================= SUBSCRIBERS  ======================================= //

    #[allow(unused)]
    vehicle_local_position_subscriber: WorkerSubscription<VehicleLocalPosition, Option<VehicleLocalPosition>>,
    vehicle_local_position_worker: Worker<Option<VehicleLocalPosition>>,

    #[allow(unused)]
    vehicle_land_detected_subscriber: WorkerSubscription<VehicleLandDetected, Option<bool>>,
    vehicle_land_detected_worker: Worker<Option<bool>>,

    // ======================================= PUBLISHERS  ======================================= //

    offboard_control_mode_publisher: Publisher<OffboardControlMode>,
    trajectory_setpoint_publisher: Publisher<TrajectorySetpoint>,

    // ======================================= STATE  ======================================= //

    vehicle_trajectory_setpoint: Arc<Mutex<Option<TrajectorySetpoint>>>,
    service_done: Arc<Mutex<Option<bool>>>,
}

impl OffboardControlNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("offboard_control").unwrap();

        let vehicle_command_client = node.create_client::<VehicleCommandSrv>("/fmu/vehicle_command").unwrap();

        while !vehicle_command_client.service_is_ready()? {
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        let vehicle_local_position_worker = node.create_worker(None);
        let vehicle_local_position_subscriber = vehicle_local_position_worker
            .create_subscription::<VehicleLocalPosition, _>(
                "/fmu/out/vehicle_local_position_v1"
                .keep_last(1).best_effort().transient_local(),
                move |data: &mut Option<VehicleLocalPosition>, msg: VehicleLocalPosition| {
                    *data = Some(msg);
                },
            )
            .unwrap();

        let vehicle_land_detected_worker = node.create_worker(None);
        let vehicle_land_detected_subscriber = vehicle_land_detected_worker
            .create_subscription::<VehicleLandDetected, _>(
                "/fmu/out/vehicle_land_detected"
                .keep_last(1).best_effort().transient_local(),
                move |data: &mut Option<bool>, msg: VehicleLandDetected| {
                    *data = Some(msg.landed);
                },
            )
            .unwrap();

        let offboard_control_mode_publisher = node.create_publisher::<OffboardControlMode>("/fmu/in/offboard_control_mode")?;
        let trajectory_setpoint_publisher = node.create_publisher::<TrajectorySetpoint>("/fmu/in/trajectory_setpoint")?;

        let vehicle_trajectory_setpoint = Arc::new(Mutex::new(None));
        let service_done = Arc::new(Mutex::new(None));

        Ok(Self { 
            node,
            vehicle_command_client,
            vehicle_local_position_subscriber, 
            vehicle_local_position_worker, 
            vehicle_land_detected_subscriber,
            vehicle_land_detected_worker,
            offboard_control_mode_publisher,
            trajectory_setpoint_publisher,
            vehicle_trajectory_setpoint,
            service_done,
        })
    }

    fn request_vehicle_command(&self, command: u16, param1: f32, param2: f32) -> Result<(), rclrs::RclrsError> {

        let now = self.node.get_clock().now().nsec / 1000;

        let request = VehicleCommand_Request {
            request: VehicleCommandMsg {
                param1: param1,
                param2: param2,
                command: command as u32,
                target_system: 1,
                target_component: 1,
                source_system: 1,
                source_component: 1,
                from_external: true,
                timestamp: now as u64,
                ..Default::default()
            }
        };

        let service_done = Arc::clone(&self.service_done);
        let logger = self.node.name().to_string();

        self.vehicle_command_client
        .call_then(&request, move |response: VehicleCommand_Response| {
            match response.reply.result {
                VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED => log!(&logger, "command accepted"),
                VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED => log_warn!(&logger, "command temporarily rejected"),
                VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED => log_warn!(&logger, "command denied"),
                VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED => log_warn!(&logger, "command unsupported"),
                VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED => log_warn!(&logger, "command failed"),
                VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS => log_warn!(&logger, "command in progress"),
                VehicleCommandAck::VEHICLE_CMD_RESULT_CANCELLED => log_warn!(&logger, "command cancelled"),
                _ => log_warn!(&logger, "command reply unknown"),
            };
            *service_done.lock().unwrap() = Some(true);
        })
        .unwrap();

        Ok(())
    }

    fn publish_offboard_control_mode(&self) -> Result<(), rclrs::RclrsError> {

        let now = self.node.get_clock().now().nsec / 1000;

        let msg = OffboardControlMode {
            position: true,
            velocity: false,
            acceleration: false,
            attitude: false,
            body_rate: false,
            thrust_and_torque: false,
            direct_actuator: false,
            timestamp: now as u64,
        };

        let _ = self.offboard_control_mode_publisher.publish(&msg);
        Ok(())
    }

    fn publish_trajectory_setpoint(&self) -> Result<(), rclrs::RclrsError> {

        if let Some(msg) = &*self.vehicle_trajectory_setpoint.lock().unwrap() {
            self.trajectory_setpoint_publisher.publish(msg);
        }
        Ok(())
    }

    fn switch_to_offboard_mode(&self) -> Result<(), rclrs::RclrsError> {

        self.request_vehicle_command(VehicleCommandMsg::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        Ok(())
    }

    fn arm(&self) -> Result<(), rclrs::RclrsError> {

        self.request_vehicle_command(VehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        Ok(())
    }

    fn disarm(&self) -> Result<(), rclrs::RclrsError> {

        self.request_vehicle_command(VehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        Ok(())
    }

    fn land(&self) -> Result<(), rclrs::RclrsError> {

        self.request_vehicle_command(VehicleCommandMsg::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
        Ok(())
    }

    fn dist(&self, p1: [f32; 3], p2: [f32; 3]) -> f32 {
        let dx = p2[0] - p1[0];
        let dy = p2[1] - p1[1];
        (dx*dx + dy*dy).sqrt()
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let node = OffboardControlNode::new(&executor).unwrap();

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(100));

        let now = node.node.get_clock().now().nsec / 1000;

        // test node state updates 

        let msg = TrajectorySetpoint {
            timestamp: now as u64,
            position: [0.0, 0.0, -5.0],
            ..Default::default()
        };
        *node.vehicle_trajectory_setpoint.lock().unwrap() = Some(msg);

        if let Some(s) = &*node.vehicle_trajectory_setpoint.lock().unwrap() {
            println!("{s:?}");
        }

        if let Some(s) = &*node.service_done.lock().unwrap() {
            println!("{s:?}");
        }

        // test vehicle_local_position subscriber 

        let _ = node.vehicle_local_position_worker.run(|data: &mut Option<VehicleLocalPosition>| {
            if let Some(data) = data {
                println!("{data:?}");
            } else {
                println!("No message available yet.");
            }
        });

        // test vehicle_land_detected subscriber 

        let _ = node.vehicle_land_detected_worker.run(|data: &mut Option<bool>| {
            if let Some(data) = data {
                println!("{data:?}");
            } else {
                println!("No message available yet.");
            }
        });

        // test publishers (ros2 topic echo ... to check in another terminal)

        node.publish_offboard_control_mode();
        node.publish_trajectory_setpoint();

        node.switch_to_offboard_mode();
    });

    executor.spin(SpinOptions::default()).first_error()
}
