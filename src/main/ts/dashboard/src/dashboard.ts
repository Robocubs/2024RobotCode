import { DashboardThemes, darkTheme } from '@frc-web-components/fwc/themes';
import { provide } from '@lit/context';
import { LitElement, css, html } from 'lit';
import { customElement } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from './network-tables/network-tables';
import { ensureSendableChooserInitialized } from './network-tables/util';
import { globalStylesCss } from './styles/styles';

@customElement('team1701-dashboard')
export class Dashboard extends LitElement {
  @provide({ context: nt4Context })
  private nt = new NetworkTables('localhost');

  constructor() {
    super();

    const themes = new DashboardThemes();
    themes.addThemeRules('dark', darkTheme);
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
            .pose=${this.nt.$pose2d('/AdvantageKit/RealOutputs/RobotState/Pose2d', [])}
          ></frc-field-robot>
          <frc-field-path
            color="red"
            opacity="1"
            .poses=${this.nt.$unit8Array('/AdvantageKit/RealOutputs/Autonomous/PathPose2ds', [])}
          ></frc-field-path>
        </frc-field>
        <team1701-match-timer timer=${this.nt.$value('/AdvantageKit/RealOutputs/RobotState/MatchTime', '0')}></team1701-match-timer>
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
