import { provide } from '@lit/context';
import { LitElement, TemplateResult, css, html } from 'lit';
import { customElement, state } from 'lit/decorators.js';
import { choose } from 'lit/directives/choose.js';
import { PageChangeDetail } from './components';
import { NetworkTables, nt4Context } from './network-tables/network-tables';
import { globalStylesCss } from './styles/styles';

@customElement('team1701-dashboard')
export class Dashboard extends LitElement {
  @provide({ context: nt4Context }) private nt = new NetworkTables(document.location.hostname);
  @state() page = new URLSearchParams(window.location.search).get('page') ?? 'Field';

  protected firstUpdated(): void {
    this.nt.bindConnection(this.renderRoot.querySelector('#dashboard-root')!);
  }

  onPageChange(event: CustomEvent<PageChangeDetail>) {
    this.page = event.detail.page;
    window.history.pushState({}, '', `?page=${this.page}`);
  }

  render(): TemplateResult {
    return html`
      <div id="dashboard-root" class="flex flex-row gap-4 h-full">
        <team1701-side-bar class="flex w-72"></team1701-side-bar>
        <div class="flex flex-col w-1 grow h-full gap-4">
          <team1701-top-bar
            class="w-full"
            page="${this.page}"
            isSimulation="${this.nt.$value('/SmartDashboard/IsSimulation', false)}"
            @pageChange="${this.onPageChange}"
          ></team1701-top-bar>
          <div class="w-full h-full">
            ${choose(
              this.page,
              [
                ['Operator', () => html`<team1701-operator-view></team1701-operator-view>`],
                ['Diagnostic', () => html`<team1701-diagnostic-view></team1701-diagnostic-view>`],
                ['Simulation', () => html`<team1701-simulation-view></team1701-simulation-view>`],
              ],
              () => html`<team1701-field-view></team1701-field-view>`
            )}
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
