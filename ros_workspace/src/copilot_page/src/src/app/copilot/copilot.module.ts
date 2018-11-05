import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { MaterialModule } from '../material.module';
import { SharedModule } from '../shared/shared.module';
import { ChartModule } from 'angular2-chartjs';

import { CopilotComponent } from './copilot.component';
import { MainCameraComponent } from './copilot-components/main-camera/main-camera.component';
import { SecondaryCameraComponent } from './copilot-components/secondary-camera/secondary-camera.component';
import { TimerComponent } from './copilot-components/timer/timer.component';
import { ThrusterSensativityComponent } from './copilot-components/thruster-sensativity/thruster-sensativity.component';
import { DepthChartComponent } from './copilot-components/depth-chart/depth-chart.component';
import { VelocityGraphComponent } from './copilot-components/velocity-graph/velocity-graph.component';
import { ToolsComponent } from './copilot-components/tools/tools.component';
import { ThrustersComponent } from './copilot-components/thrusters/thrusters.component';
import { SystemsComponent } from './copilot-components/systems/systems.component';
import { TelemetrydataComponent } from './copilot-components/telemetry-data/telemetrydata.component';

@NgModule({
  imports: [
    CommonModule,
    MaterialModule,
    SharedModule,
    ChartModule
  ],
  declarations: [
    CopilotComponent,
    MainCameraComponent,
    SecondaryCameraComponent,
    TimerComponent,
    ThrusterSensativityComponent,
    DepthChartComponent,
    VelocityGraphComponent,
    ToolsComponent,
    ThrustersComponent,
    SystemsComponent,
    TelemetrydataComponent,
  ],
  exports: [
    MainCameraComponent,
    SecondaryCameraComponent,
    TimerComponent,
    ThrusterSensativityComponent,
    DepthChartComponent,
    VelocityGraphComponent,
    ToolsComponent,
    ThrustersComponent,
    SystemsComponent,
    TelemetrydataComponent
  ]
})
export class CopilotModule { }
