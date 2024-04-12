import { consume } from '@lit/context';
import { LitElement, TemplateResult, css, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { classMap } from 'lit/directives/class-map.js';
import { when } from 'lit/directives/when.js';
import { NetworkTables, nt4Context } from '../network-tables/network-tables';
import { ensureSendableChooserInitialized } from '../network-tables/util';
import { globalStylesCss } from '../styles/styles';

export interface PageChangeDetail {
  page: 'Field' | 'Operator' | 'Diagnostic' | 'Simulation';
}

@customElement('team1701-top-bar')
export class TopBar extends LitElement {
  @consume({ context: nt4Context }) private nt!: NetworkTables;
  @property({ type: String }) page = 'Field';
  @property({ type: Boolean }) isSimulation = false;

  protected firstUpdated(): void {
    this.nt.bindConnection(this.renderRoot.querySelector('#top-bar-root')!);
    ensureSendableChooserInitialized(this.nt, '/SmartDashboard/Auto Mode');
  }

  #dispatchPageChange(page: PageChangeDetail['page']): void {
    this.dispatchEvent(
      new CustomEvent('pageChange', {
        detail: {
          page,
        } satisfies PageChangeDetail,
        bubbles: true,
        composed: true,
      })
    );
  }

  render(): TemplateResult {
    return html`
      <div id="top-bar-root" class="flex flex-row justify-between w-full">
        <div class="flex flex-row gap-2">
          <frc-sendable-chooser source-key="/SmartDashboard/Auto Mode"></frc-sendable-chooser>
          <team1701-incrementable-number
            source-key="/SmartDashboard/ShooterSpeakerRotationOffsetRadians"
            label="Speaker Offset"
            step="0.005"
          ></team1701-incrementable-number>
        </div>
        <ul class="flex flex-wrap gap-2 cursor-pointer select-none text-md font-medium text-center text-white border-b-2 border-primary h-full">
          <li>
            <div @click="${() => this.#dispatchPageChange('Field')}" class="${classMap(this.tabClass('Field'))}">Field</div>
          </li>
          <li>
            <div @click="${() => this.#dispatchPageChange('Operator')}" class="${classMap(this.tabClass('Operator'))}">Operator</div>
          </li>
          <li>
            <div @click="${() => this.#dispatchPageChange('Diagnostic')}" class="${classMap(this.tabClass('Diagnostic'))}">Diagnostic</div>
          </li>
          ${when(
            this.isSimulation,
            () =>
              html`<li>
                <div @click="${() => this.#dispatchPageChange('Simulation')}" class="${classMap(this.tabClass('Simulation'))}">Simulation</div>
              </li>`
          )}
        </ul>
      </div>
    `;
  }

  tabClass(tab: string) {
    const commonClasses = {
      'inline-block': true,
      'p-4': true,
      'rounded-t-lg': true,
    };
    return this.page === tab
      ? {
          ...commonClasses,
          'bg-primary': true,
        }
      : {
          ...commonClasses,
          'hover:bg-neutral-700': true,
          'hover:text-neutral-200': true,
        };
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-top-bar': TopBar;
  }
}
