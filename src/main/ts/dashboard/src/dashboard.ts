import { consume } from '@lit/context';
import { LitElement, css, html } from 'lit';
import { customElement } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from './network-tables/network-tables';
import { ensureSendableChooserInitialized } from './network-tables/util';

@customElement('team1701-dashboard')
export class Dashboard extends LitElement {
  @consume({ context: nt4Context })
  private nt!: NetworkTables;

  createRenderRoot() {
    return this;
  }

  protected firstUpdated(): void {
    ensureSendableChooserInitialized(this.nt, '/SmartDashboard/Auto Mode');
  }

  render() {
    return html`
      <div>
        <frc-sendable-chooser source-key="/SmartDashboard/Auto Mode"></frc-sendable-chooser>
        <frc-basic-fms-info source-key="/FMSInfo"></frc-basic-fms-info>
        <frc-toggle-button label="Toggle Button" source-key="/Dashboard/Toggled"></frc-toggle-button>
      </div>
      <frc-field crop-left=".1" crop-right=".9" rotation-unit="deg">
        <frc-field-robot
          color="blue"
          opacity="1"
          rotation-unit="rad"
          width="0.5"
          length="0.6"
          .pose=${this.nt.$pose2d('/AdvantageKit/RealOutputs/PoseEstimator/Pose2d', [])}
        ></frc-field-robot>
      </frc-field>
    `;
  }

  static styles = css`
    div {
      display: flex;
      gap: 15px;
      align-items: start;
    }

    frc-field {
      width: 500px;
      height: 300px;
    }
  `;
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-dashboard': Dashboard;
  }
}
