import {Component, OnInit} from '@angular/core';

declare var Snap: any;
declare var mina: any;

@Component({
  selector: 'app-velocity-graph',
  templateUrl: './velocity-graph.component.html',
  styleUrls: ['./velocity-graph.component.css']
})
export class VelocityGraphComponent implements OnInit {

  name = 'Velocity Graph';

  check = 1;
  frame;
  background;
  hand1;

  ngOnInit() {
    // @ts-ignore
    this.frame = Snap(".frame");
    this.background = this.frame.circle(96, 96, 90).attr({
      fill: "#ffffff",
      stroke: "#df5b4d",
      strokeWidth: 4
    });

    this.hand1 = this.frame.rect(87, 54, 18, 66, 9).attr({fill: "#344d5a"});
    this.hand1.animate({transform: "r" + 24 + "," + 96 + "," + 96}, 500, mina.elastic);
    console.log(this.hand1);
    window.setInterval(this.update, 1000)
  }

  update() {
    this.check ++;
    this.check = (this.check > 60) ? this.check = 1 : this.check;
    // @ts-ignore
    this.hand1.animate({transform: "r" + this.check * 6 + "," + 96 + "," + 96}, 500, mina.elastic);
    console.log("Update " + this.check);
  }
}
