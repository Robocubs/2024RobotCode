import { consume } from '@lit/context';
import { LitElement, css, html } from 'lit';
import { customElement } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from '../network-tables/network-tables';
import { globalStylesCss } from '../styles/styles';

@customElement('team1701-main-view')
export class MainView extends LitElement {
  @consume({ context: nt4Context })
  private nt!: NetworkTables;

  protected firstUpdated(): void {
    this.nt.bindConnection(this.renderRoot.querySelector('#main-view-root')!);
  }

  render() {
    return html`<div id="main-view-root" class="flex flex-col gap-4 h-full">
      <div class="flex flex-row gap-4">
        <frc-sendable-chooser source-key="/SmartDashboard/Auto Mode"></frc-sendable-chooser>
      </div>
      <frc-field class="w-full h-full" crop-left=".1" crop-right=".9" rotation-unit="deg">
        <frc-field-robot
          color="blue"
          opacity="1"
          rotation-unit="rad"
          width="0.5"
          length="0.6"
          .pose=${this.nt.$doubleArray('/AdvantageKit/RealOutputs/RobotState/Pose2d', [])}
        ></frc-field-robot>
        <frc-field-path
          color="violet"
          opacity="1"
          .poses=${this.nt.$doubleArray('/AdvantageKit/RealOutputs/Autonomous/PathPose2ds', [])}
        ></frc-field-path>
      </frc-field>
    </div>`;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-main-view': MainView;
  }
}
