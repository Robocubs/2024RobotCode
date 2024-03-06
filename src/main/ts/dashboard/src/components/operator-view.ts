import { consume } from '@lit/context';
import { LitElement, TemplateResult, css, html } from 'lit';
import { customElement, state } from 'lit/decorators.js';
import { map } from 'lit/directives/map.js';
import { NetworkTables, nt4Context } from '../network-tables/network-tables';
import { globalStylesCss } from '../styles/styles';

interface StreamDeckState {
  Button: {
    [key: string]: StreamDeckButton;
  };
}

interface StreamDeckButton {
  Icon: string;
  Key: string;
  Label: string;
  Selected: boolean;
}

@customElement('team1701-operator-view')
export class OperatorView extends LitElement {
  @consume({ context: nt4Context }) private nt!: NetworkTables;
  @state() buttons: (StreamDeckButton | undefined)[] = [];

  protected firstUpdated(): void {
    this.nt.addKeyListener(
      '/StreamDeck',
      (_, value) => {
        const state = value as StreamDeckState;
        if (!state || !Object.hasOwn(state, 'Button')) {
          this.buttons = [];
          return;
        }

        this.buttons = Array.from({ length: 15 }, (_, i) => state.Button[`${i}`]);
      },
      true
    );
  }

  render(): TemplateResult {
    return html`<div id="operator-buttons-root" class="grid h-full grid-rows-3 grid-cols-5 gap-4">
      ${map(this.buttons, (button) => {
        if (!button) {
          return html`<div class="h-28"></div>`;
        }

        return html`<team1701-operator-button label="${button.Label}" key="${button.Key}" ?selected="${button.Selected}"></team1701-operator-button>`;
      })}
    </div>`;
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-operator-view': OperatorView;
  }
}
