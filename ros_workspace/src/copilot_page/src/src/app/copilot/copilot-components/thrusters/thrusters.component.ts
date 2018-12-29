import { Component, OnInit } from '@angular/core';
import * as $ from 'jquery';
import 'jquery-knob';
import { HorizontalDriveService } from '../../../services/subscribers/horizontal-drive.service';
import { HorizontalDriveModel } from '../../../services/data-models/horizontal-drive.model';
import { VerticalDriveService } from '../../../services/subscribers/vertical-drive.service';
import { VerticalDriveModel } from '../../../services/data-models/vertical-drive.model';

@Component({
  selector: 'app-thrusters',
  templateUrl: './thrusters.component.html',
  styleUrls: ['./thrusters.component.css']
})
export class ThrustersComponent implements OnInit {
    name = 'ROV Thrusters';

    thruster1: number;
    thruster2: number;
    thruster3: number;
    thruster4: number;
    thruster5: number;
    thruster6: number;

    constructor(private horizontaldriveService: HorizontalDriveService, private verticaldriveService: VerticalDriveService) {}

    thrustToPercent(msg) {
        msg = msg / 10;
        return msg;
    }

    ngOnInit() {
        $(".dial").knob({
            'min': -100,
            'max': 100,
            'width': '100%',
            'height': '100%',
            'lineCap': 'round',
            'readOnly': true,
            'fgColor': '#4287e0',
            'bgColor': '#204d7a'
        });
        this.horizontaldriveService.initialize();
        this.horizontaldriveService.getData().subscribe((msg: HorizontalDriveModel) => {
            this.thruster1 = this.thrustToPercent(msg.t1);
            this.thruster2 = this.thrustToPercent(msg.t2);
            this.thruster3 = this.thrustToPercent(msg.t3);
            this.thruster4 = this.thrustToPercent(msg.t4);
            $("#thruster1").val(this.thruster1).trigger('change');
            $("#thruster2").val(this.thruster2).trigger('change');
            $("#thruster3").val(this.thruster3).trigger('change');
            $("#thruster4").val(this.thruster4).trigger('change');
        });
        this.verticaldriveService.initialize();
        this.verticaldriveService.getData().subscribe((msg: VerticalDriveModel) => {
            this.thruster5 = this.thrustToPercent(msg.t1);
            this.thruster6 = this.thrustToPercent(msg.t2);
            $("#thruster5").val(this.thruster5).trigger('change');
            $("#thruster6").val(this.thruster6).trigger('change');
        });
    }



}
