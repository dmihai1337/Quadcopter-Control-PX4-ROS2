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

#[derive(Debug, PartialEq)]
enum State {
    Init,
    OffboardRequested,
    WaitForStableOffboardMode,
    ArmRequested,
    Ascend,
    Point1, Point2, Point3, Point4,
    Land
}

pub struct OffboardControlNode {

    node: Node,

    // ======================================= CLIENT  ======================================= //

    vehicle_command_client: Client<VehicleCommandSrv>,

    // ======================================= SUBSCRIBERS  ======================================= //

    vehicle_local_position_subscriber: Subscription<VehicleLocalPosition>,
    vehicle_land_detected_subscriber: Subscription<VehicleLandDetected>,

    // ======================================= PUBLISHERS  ======================================= //

    offboard_control_mode_publisher: Publisher<OffboardControlMode>,
    trajectory_setpoint_publisher: Publisher<TrajectorySetpoint>,

    // ======================================= STATE  ======================================= //

    vehicle_local_position: Arc<Mutex<Option<[f32; 3]>>>,
    vehicle_land_detected: Arc<Mutex<Option<bool>>>,
    vehicle_trajectory_setpoint: Arc<Mutex<Option<[f32; 3]>>>,
    service_done: Arc<Mutex<Option<bool>>>,
    service_result: Arc<Mutex<Option<u8>>>,
    state: Arc<Mutex<Option<State>>>
}

impl OffboardControlNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("offboard_control").unwrap();

        // ======================================= CLIENT  ======================================= //

        let vehicle_command_client = node.create_client::<VehicleCommandSrv>("/fmu/vehicle_command").unwrap();

        log!(node.logger().once(), "Starting Offboard Control Mission with PX4 services");
        log!(node.logger().once(), "Waiting for /fmu/vehicle_command service");

        while !vehicle_command_client.service_is_ready()? {
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        // ======================================= SUBSCRIBERS  ======================================= //

        // we need 2 shared pointers for each of the data points we are subscribed to
        // 1 for instantiation through Ok(Self{}) at the end, 1 for the subscription callback
        let vehicle_local_position = Arc::new(Mutex::new(Some([0.0, 0.0, 0.0])));
        let vehicle_local_position_callback = Arc::clone(&vehicle_local_position);

        let vehicle_land_detected = Arc::new(Mutex::new(Some(true)));
        let vehicle_land_detected_callback = Arc::clone(&vehicle_land_detected);

        let vehicle_local_position_subscriber = node.create_subscription(
            "/fmu/out/vehicle_local_position_v1"
            .keep_last(1).best_effort().transient_local(),
            move |msg: VehicleLocalPosition| {
                *vehicle_local_position_callback.lock().unwrap() = Some([msg.x, msg.y, msg.z]);
            },
        )
        .unwrap();

        let vehicle_land_detected_subscriber = node.create_subscription(
            "/fmu/out/vehicle_land_detected"
            .keep_last(1).best_effort().transient_local(),
            move |msg: VehicleLandDetected| {
                *vehicle_land_detected_callback.lock().unwrap() = Some(msg.landed);
            },
        )
        .unwrap();

        // ======================================= PUBLISHERS  ======================================= //

        let offboard_control_mode_publisher = node.create_publisher::<OffboardControlMode>("/fmu/in/offboard_control_mode")?;
        let trajectory_setpoint_publisher = node.create_publisher::<TrajectorySetpoint>("/fmu/in/trajectory_setpoint")?;

        // ======================================= STATE  ======================================= //

        let now = node.get_clock().now().nsec / 1000;
        let vehicle_trajectory_setpoint = Arc::new(Mutex::new(Some([0.0, 0.0, -5.0])));
        let service_done = Arc::new(Mutex::new(Some(false)));
        let service_result = Arc::new(Mutex::new(Some(VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)));
        let state = Arc::new(Mutex::new(Some(State::Init)));

        Ok(Self { 
            node,
            vehicle_command_client,
            vehicle_local_position_subscriber, 
            vehicle_land_detected_subscriber,
            offboard_control_mode_publisher,
            trajectory_setpoint_publisher,
            vehicle_local_position,
            vehicle_land_detected,
            vehicle_trajectory_setpoint,
            service_done,
            service_result,
            state
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
        let service_result = Arc::clone(&self.service_result);
        let logger = self.node.name().to_string();

        log!(self.node.logger().once(), "Command sent");

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
            *service_result.lock().unwrap() = Some(response.reply.result);
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

        let now = self.node.get_clock().now().nsec / 1000;

        let Some(position) = &*self.vehicle_trajectory_setpoint.lock().unwrap() else { panic!("Invalid Node State Variable") };

        let msg = TrajectorySetpoint {
            timestamp: now as u64,
            position: *position,
            ..Default::default()
        };

        self.trajectory_setpoint_publisher.publish(msg);
        Ok(())
    }

    fn switch_to_offboard_mode(&self) -> Result<(), rclrs::RclrsError> {

        log!(self.node.logger().once(), "requesting switch to Offboard mode");
        self.request_vehicle_command(VehicleCommandMsg::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        Ok(())
    }

    fn arm(&self) -> Result<(), rclrs::RclrsError> {

        log!(self.node.logger().once(), "requesting arm");
        self.request_vehicle_command(VehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        Ok(())
    }

    fn disarm(&self) -> Result<(), rclrs::RclrsError> {

        log!(self.node.logger().once(), "requesting disarm");
        self.request_vehicle_command(VehicleCommandMsg::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        Ok(())
    }

    fn land(&self) -> Result<(), rclrs::RclrsError> {

        log!(self.node.logger().once(), "requesting land");
        self.request_vehicle_command(VehicleCommandMsg::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
        Ok(())
    }

    fn close_to_target(&self) -> bool {

        let Some(current_position) = &*self.vehicle_local_position.lock().unwrap() else { panic!("Invalid Node State Variable") };
        let Some(goal_position) = &*self.vehicle_trajectory_setpoint.lock().unwrap() else { panic!("Invalid Node State Variable") };

        let dx = goal_position[0] - current_position[0];
        let dy = goal_position[1] - current_position[1];
        (dx*dx + dy*dy).sqrt() < 0.035
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env().unwrap().create_basic_executor();
    let node = OffboardControlNode::new(&executor).unwrap();

    let mut num_steps: u8 = 0;

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(100));

        let now = node.node.get_clock().now().nsec / 1000;

        // get mutex guard of state variable -> read & write 
        let mut state_guard = node.state.lock().unwrap();
        let state = state_guard.take().expect("Invalid Node State");

        if state != State::Land {
            // offboard_control_mode needs to be paired with trajectory_setpoint
            node.publish_offboard_control_mode();
            node.publish_trajectory_setpoint();
        }

        // get read lock for needed state variables
        let Some(service_done) = &*node.service_done.lock().unwrap() else { panic!("Invalid Node State Variable") };
        let Some(service_result) = &*node.service_result.lock().unwrap() else { panic!("Invalid Node State Variable") };

        match state {
            State::Init => {
                node.switch_to_offboard_mode();
                *state_guard = Some(State::OffboardRequested);
            },
            State::OffboardRequested => {
                if *service_done {
                    if *service_result == 0 {
                        log!(node.node.logger().once(), "Entered offboard mode");
                        *state_guard = Some(State::WaitForStableOffboardMode);				
                    }
                    else {
                        log_warn!(node.node.logger().once(), "Failed to enter offboard mode, exiting");
                        std::process::exit(0);
                    }
                } 
                else {
                    *state_guard = Some(State::OffboardRequested);
                }
            },
            State::WaitForStableOffboardMode => {
                num_steps += 1;
                if num_steps > 10 {
                    node.arm();
                    *state_guard = Some(State::ArmRequested);
                }
                else {
                    *state_guard = Some(State::WaitForStableOffboardMode);
                }
            },
            State::ArmRequested => {
                if *service_done {
                    if *service_result == 0 {
                        log!(node.node.logger().once(), "vehicle is armed, taking off");
                        *state_guard = Some(State::Ascend);
                    }
                    else{
                        log_warn!(node.node.logger().once(), "Failed to arm, exiting");
                        std::process::exit(0);
                    }
                }
                else {
                    *state_guard = Some(State::ArmRequested);
                }
            },
            State::Ascend => {
                let Some(vehicle_local_position) = &*node.vehicle_local_position.lock().unwrap() else { panic!("Invalid Node State Variable") };
                if vehicle_local_position[2] < -5.0 {
                    *node.vehicle_trajectory_setpoint.lock().unwrap() = Some([10.0, 0.0, -5.0]);
                    log!(node.node.logger().once(), "Reached altitude");
                    *state_guard = Some(State::Point1);
                }
                else {
                    *state_guard = Some(State::Ascend);
                }
            },
            State::Point1 => {
                if node.close_to_target() {
                    *node.vehicle_trajectory_setpoint.lock().unwrap() = Some([10.0, 10.0, -5.0]);                    
                    log!(node.node.logger().once(), "Reached point 1");
                    *state_guard = Some(State::Point2);  
                }
                else {
                    *state_guard = Some(State::Point1);
                }
            },
            State::Point2 => {
                if node.close_to_target() {
                    *node.vehicle_trajectory_setpoint.lock().unwrap() = Some([0.0, 10.0, -5.0]);                    
                    log!(node.node.logger().once(), "Reached point 2");
                    *state_guard = Some(State::Point3);  
                }
                else {
                    *state_guard = Some(State::Point2);
                }
            },
            State::Point3 => {
                if node.close_to_target() {
                    *node.vehicle_trajectory_setpoint.lock().unwrap() = Some([0.0, 0.0, -5.0]);                    
                    log!(node.node.logger().once(), "Reached point 3");
                    *state_guard = Some(State::Point4);  
                }
                else {
                    *state_guard = Some(State::Point3);
                }
            },
            State::Point4 => {
                if node.close_to_target() {
                    log!(node.node.logger().once(), "Reached point 4");
                    node.land();                
                    *state_guard = Some(State::Land);  
                }
                else {
                    *state_guard = Some(State::Point4);
                }
            },
            State::Land => {
                let Some(vehicle_land_detected) = &*node.vehicle_land_detected.lock().unwrap() else { panic!("Invalid Node State Variable") };
                if *service_done {
                    if *service_result == 0 {
                        if *vehicle_land_detected {
                            log!(node.node.logger().once(), "vehicle has landed");
                            std::process::exit(0);
                        }
                    }
                    else {
                        log_warn!(node.node.logger().once(), "Failed to land, exiting");
                        std::process::exit(0);
                    }
                }
                *state_guard = Some(State::Land);
            },
            _ => println!("Unknown state"),
        };
    });

    executor.spin(SpinOptions::default()).first_error()
}
