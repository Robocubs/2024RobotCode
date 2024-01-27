import { DashboardThemes, darkTheme } from '@frc-web-components/fwc/themes';
import { provide } from '@lit/context';
import { LitElement, html } from 'lit';
import { customElement } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from './network-tables/network-tables';

@customElement('team1701-app')
export class App extends LitElement {
  @provide({ context: nt4Context })
  // @ts-expect-error 6133
  private nt = new NetworkTables('localhost');

  createRenderRoot() {
    return this;
  }

  constructor() {
    super();

    const themes = new DashboardThemes();
    themes.addThemeRules('dark', darkTheme);
    themes.setTheme(document.body, 'dark');
  }

  render() {
    return html`<team1701-dashboard></team1701-dashboard>`;
  }
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-app': App;
  }
}
