import { consume } from '@lit/context';
import { LitElement, TemplateResult, css, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from '../network-tables/network-tables';
import { ensureSendableChooserInitialized } from '../network-tables/util';
import { globalStylesCss } from '../styles/styles';

export interface ToggleOperatorDetail {
  toggled: boolean;
}

@customElement('team1701-top-bar')
export class TopBar extends LitElement {
  @consume({ context: nt4Context })
  private nt = new NetworkTables(document.location.hostname);

  @property({ type: Boolean }) operatorToggled = false;

  protected firstUpdated(): void {
    this.nt.bindConnection(this.renderRoot.querySelector('#top-bar-root')!);
    ensureSendableChooserInitialized(this.nt, '/SmartDashboard/Auto Mode');
  }

  onToggleOperator(event: CustomEvent<{ toggled: boolean }>): void {
    this.#dispatchToggleOperator(event.detail.toggled);
  }

  #dispatchToggleOperator(toggled: boolean): void {
    this.dispatchEvent(
      new CustomEvent('toggleOperator', {
        detail: {
          toggled,
        } satisfies ToggleOperatorDetail,
        bubbles: true,
        composed: true,
      })
    );
  }

  render(): TemplateResult {
    return html`
      <div id="top-bar-root" class="grid grid-cols-3 gap-4">
        <div class="flex justify-left"><frc-basic-fms-info class="flex items-center" source-key="/FMSInfo"></frc-basic-fms-info></div>
        <team1701-match-timer timer=${this.nt.$value('/AdvantageKit/RealOutputs/RobotState/MatchTime', '0')}></team1701-match-timer>
        <div class="flex justify-end">
          <frc-toggle-button label="Operator" ?toggled="${this.operatorToggled}" @toggle="${this.onToggleOperator}"></frc-toggle-button>
        </div>
      </div>
    `;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-top-bar': TopBar;
  }
}
