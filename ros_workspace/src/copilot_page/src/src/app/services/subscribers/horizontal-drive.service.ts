import { Injectable } from '@angular/core';
import {BehaviorSubject, Observable} from 'rxjs';
import '../../../assets/roslib.js';

@Injectable({
  providedIn: 'root'
})
export class HorizontalDriveService {
  // Horizontal thruster percentages
  // Creates object with the ROS library
  // @ts-ignore
  ros = new ROSLIB.Ros({
    // Set listen URL for ROS communication
    url : 'ws://master:9090'
  });

  // Define subject to hold data values
  horizontalDrive: BehaviorSubject<any> = new BehaviorSubject('Untouched');
  // Initializer to be called every time horizontal drive is going to be used
  initialize() {
    // Get Data from ROS horzontal drive Topic
    // @ts-ignore
    const horizontalDriveListener = new ROSLIB.Topic({
      ros: this.ros,
      name: '/rov/cmd_horizontal_vdrive',
      messageType: 'vector_drive/thrusterPercents'
    });

    // Subscribe to horizontal drive listener
    horizontalDriveListener.subscribe((message) => {
      this.horizontalDrive.next(message);
    });
  }
  // Define data getter
  getData(): Observable<any> { return this.horizontalDrive.asObservable(); }
}
