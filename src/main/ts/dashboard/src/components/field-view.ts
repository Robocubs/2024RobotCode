import { consume } from '@lit/context';
import { LitElement, css, html } from 'lit';
import { customElement } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from '../network-tables/network-tables';
import { globalStylesCss } from '../styles/styles';

@customElement('team1701-field-view')
export class FieldView extends LitElement {
  @consume({ context: nt4Context }) private nt!: NetworkTables;

  render() {
    return html`<frc-field style="align-items: top" class="grow w-full h-full" crop-left=".1" crop-right=".9" rotation-unit="deg">
      <frc-field-robot
        color="${this.nt.$allianceColor()}"
        opacity="1"
        rotation-unit="deg"
        width="0.7"
        length="0.7"
        .pose=${this.nt.$value('/SmartDashboard/Field/Robot', [])}
      ></frc-field-robot>
      <frc-field-path color="orange" opacity="1" .poses=${this.nt.$value('/SmartDashboard/Field/Path', [])}></frc-field-path>
    </frc-field>`;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-field-view': FieldView;
  }
}
