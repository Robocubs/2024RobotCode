import { provide } from '@lit/context';
import { LitElement, TemplateResult, css, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { choose } from 'lit/directives/choose.js';
import { ToggleOperatorDetail } from './components';
import { NetworkTables, nt4Context } from './network-tables/network-tables';
import { globalStylesCss } from './styles/styles';

@customElement('team1701-dashboard')
export class Dashboard extends LitElement {
  @provide({ context: nt4Context })
  private nt = new NetworkTables(document.location.hostname);

  @property({ type: String }) page = new URLSearchParams(window.location.search).get('page') ?? 'Main';

  protected firstUpdated(): void {
    this.nt.bindConnection(this.renderRoot.querySelector('#dashboard-root')!);
  }

  toggleOperator(event: CustomEvent<ToggleOperatorDetail>) {
    this.page = event.detail.toggled ? 'Operator' : 'Field';
    window.history.pushState({}, '', `?page=${this.page}`);
  }

  render(): TemplateResult {
    return html`
      <div id="dashboard-root" class="flex flex-row gap-4 h-full">
        <team1701-side-bar
          class="flex w-72"
          ?operatorToggled="${this.page === 'Operator'}"
          @toggleOperator="${this.toggleOperator}"
        ></team1701-side-bar>
        <div class="w-1 grow h-full">
          ${choose(
            this.page,
            [['Operator', () => html`<team1701-operator-view></team1701-operator-view>`]],
            () => html`<team1701-field-view></team1701-field-view>`
          )}
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
