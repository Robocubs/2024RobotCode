import { consume } from '@lit/context';
import { LitElement, css, html } from 'lit';
import { customElement } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from '../network-tables/network-tables';
import { globalStylesCss } from '../styles/styles';

@customElement('team1701-diagnostic-view')
export class DiagnosticView extends LitElement {
  @consume({ context: nt4Context }) private nt!: NetworkTables;

  protected firstUpdated(): void {
    this.nt.bindConnection(this.renderRoot.querySelector('#diagnostic-view-root')!);
  }

  render() {
    return html`<div id="diagnostic-view-root" class="grid grid-cols-3 gap-4">
      <frc-robot-subsystem class="font-sans" label="Drive" source-key="/SmartDashboard/Drive"></frc-robot-subsystem>
      <team1701-field-view class="col-span-2 row-span-2 xl:row-span-4 2xl:row-span-5"></team1701-field-view>
      <frc-robot-subsystem class="font-sans" label="Shooter" source-key="/SmartDashboard/Shooter"></frc-robot-subsystem>
      <frc-robot-subsystem class="font-sans" label="Indexer" source-key="/SmartDashboard/Indexer"></frc-robot-subsystem>
      <frc-robot-subsystem class="font-sans" label="Intake" source-key="/SmartDashboard/Intake"></frc-robot-subsystem>
      <frc-robot-subsystem class="font-sans" label="Climb" source-key="/SmartDashboard/Climb"></frc-robot-subsystem>
    </div>`;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-diagnostic-view': DiagnosticView;
  }
}
