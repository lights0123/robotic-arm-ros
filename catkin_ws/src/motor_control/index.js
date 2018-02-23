#!/usr/bin/node
const PivotPi = require('node-pivotpi');
let pivotpi = new PivotPi(0x41, 60);
const motorSettings = {
	dcs:['M1','M2','M3','M4'],
	address:0x60
};
/* Motor Configuration:
 *
 * 1 0
 *
 * 3 2
 */

const motorHat = require('motor-hat')(motorSettings).init();
const rosnodejs = require('rosnodejs');
const Twist = rosnodejs.require("geometry_msgs").msg.Twist;
rosnodejs.initNode('/motor_control').then(nh=>{
	let timeoutTimer = 0;
	const motionSub = nh.subscribe('/cmd_vel', Twist, vel_msg => {
		let speed = vel_msg.linear.x;
		let turn = vel_msg.angular.z;
		let abs = Math.abs;
		if(turn < 0){
			motorHat.dcs[1].setSpeed(abs(speed)*100,()=>motorHat.dcs[1].run(speed < 0 ? 'fwd' : 'back',()=>null));
			motorHat.dcs[3].setSpeed(abs(speed)*100,()=>motorHat.dcs[3].run(speed < 0 ? 'fwd' : 'back',()=>null));

			motorHat.dcs[0].setSpeed(abs(speed)*100,()=>motorHat.dcs[0].run(speed > 0 ? 'fwd' : 'back',()=>null));
			motorHat.dcs[2].setSpeed(abs(speed)*100,()=>motorHat.dcs[2].run(speed > 0 ? 'fwd' : 'back',()=>null));
		} else if(turn > 0){
			motorHat.dcs[1].setSpeed(abs(speed)*100,()=>motorHat.dcs[1].run(speed > 0 ? 'fwd' : 'back',()=>null));
			motorHat.dcs[3].setSpeed(abs(speed)*100,()=>motorHat.dcs[3].run(speed > 0 ? 'fwd' : 'back',()=>null));

			motorHat.dcs[0].setSpeed(abs(speed)*100,()=>motorHat.dcs[0].run(speed < 0 ? 'fwd' : 'back',()=>null));
			motorHat.dcs[2].setSpeed(abs(speed)*100,()=>motorHat.dcs[2].run(speed < 0 ? 'fwd' : 'back',()=>null));
		} else {
			motorHat.dcs[1].setSpeed(abs(speed)*100,()=>motorHat.dcs[1].run(speed > 0 ? 'fwd' : 'back',()=>null));
			motorHat.dcs[3].setSpeed(abs(speed)*100,()=>motorHat.dcs[3].run(speed > 0 ? 'fwd' : 'back',()=>null));
			motorHat.dcs[0].setSpeed(abs(speed)*100,()=>motorHat.dcs[0].run(speed > 0 ? 'fwd' : 'back',()=>null));
			motorHat.dcs[2].setSpeed(abs(speed)*100,()=>motorHat.dcs[2].run(speed > 0 ? 'fwd' : 'back',()=>null));
		}
		clearTimeout(timeoutTimer);
		timeoutTimer = setTimeout(()=>{

		},1000);
	});
	let servoSubs = [];
	for(let i = 1; i <=5; i++){
		servoSubs+=nh.subscribe('robot_arm/joint'+i,'std_msgs/UInt16',msg=>{
			pivotpi.angleMicroseconds(i-1, msg.data);
		});
	}
});

