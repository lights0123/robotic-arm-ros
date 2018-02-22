#!/usr/bin/node
const PivotPi = require('node-pivotpi');
const motorSettings = {
	dcs:['M1','M2','M3','M4'],
	address:0x60
};
const motorHat = require('motor-hat')(motorSettings);
const rosnodejs = require('rosnodejs');
rosnodejs.initNode('/motor_control').then(()=>{
	const nh = rosnodejs.nh;
});

