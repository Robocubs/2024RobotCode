import { DashboardThemes, darkTheme } from '@frc-web-components/fwc/themes';
import { provide } from '@lit/context';
import { LitElement, css, html } from 'lit';
import { customElement } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from './network-tables/network-tables';
import { ensureSendableChooserInitialized } from './network-tables/util';
import { globalStylesCss } from './styles/styles';
import { customDarkTheme } from './styles/themes';

@customElement('team1701-dashboard')
export class Dashboard extends LitElement {
  @provide({ context: nt4Context })
  private nt = new NetworkTables('localhost');

  constructor() {
    super();

    const themes = new DashboardThemes();
    themes.addThemeRules('dark', { ...darkTheme, ...customDarkTheme });
    themes.setTheme(document.body, 'dark');
  }

  protected firstUpdated(): void {
    this.nt.bindConnection(this.renderRoot.querySelector('#dashboard-root')!);
    ensureSendableChooserInitialized(this.nt, '/SmartDashboard/Auto Mode');
  }

  render() {
    return html`
      <div id="dashboard-root" class="flex flex-col gap-4 h-full">
        <div class="grid grid-cols-3 gap-4">
          <div><frc-sendable-chooser source-key="/SmartDashboard/Auto Mode"></frc-sendable-chooser></div>
          <div class="flex justify-center"><frc-basic-fms-info class="flex items-center" source-key="/FMSInfo"></frc-basic-fms-info></div>
          <div class="flex justify-end">
            <frc-toggle-button label="Toggle Button" source-key="/Dashboard/Toggled"></frc-toggle-button>
          </div>
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
        <div class="flex flex-row items-end grid gap-x-8 gap-y-4 grid-cols-3">
          <frc-toggle-button
            style="width:200px; height:100px; font-size:20px;"
            label="Stop Intake"
            source-key="/SmartDashboard/Controls/StopIntake"
          ></frc-toggle-button>
          <team1701-match-timer timer=${this.nt.$value('/AdvantageKit/RealOutputs/RobotState/MatchTime', '0')}></team1701-match-timer>
          <div class="justify-self-end">
            <frc-toggle-button
              style="width:200px; height:100px; font-size:20px;"
              label="Reverse"
              source-key="/SmartDashboard/Controls/Reverse"
            ></frc-toggle-button>
          </div>
          <frc-toggle-button
            style="width:200px; height:100px; font-size:20px;"
            label="Arm Up"
            source-key="/SmartDashboard/Controls/ArmUp"
          ></frc-toggle-button>
          <div></div>
          <div class="justify-self-end">
            <frc-toggle-button
              style="width:200px; height:100px; font-size:20px;"
              label="Arm Down"
              source-key="/SmartDashboard/Controls/ArmDown"
            ></frc-toggle-button>
          </div>
        </div>
      </div>
    `;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-dashboard': Dashboard;
  }
}
