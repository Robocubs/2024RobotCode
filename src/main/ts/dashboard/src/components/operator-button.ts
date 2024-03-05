import { consume } from '@lit/context';
import { LitElement, TemplateResult, css, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { NetworkTables, nt4Context } from '../network-tables/network-tables';
import { globalStylesCss } from '../styles/styles';

@customElement('team1701-operator-button')
export class OperatorButton extends LitElement {
  @consume({ context: nt4Context }) private nt!: NetworkTables;
  @property({ type: String }) key = '';
  @property({ type: String }) label = 'Button';
  @property({ type: Boolean }) selected = false;

  onMouseDown(event: Event): void {
    if (this.key) {
      this.nt.setValue(this.key, true);
      event.preventDefault();
    }
  }

  onMouseUp(): void {
    this.nt.setValue(this.key, false);
  }

  render(): TemplateResult {
    return html`
      <button
        class="${this.selected ? 'toggled' : ''} cursor-pointer"
        @mousedown="${this.onMouseDown}"
        @touchstart="${this.onMouseDown}"
        @mouseup="${this.onMouseUp}"
        @touchend="${this.onMouseUp}"
        @touchcancel="${this.onMouseUp}"
      >
        ${this.label}
      </button>
    `;
  }

  static styles = [
    globalStylesCss,
    css`
      button {
        width: 100%;
        height: 100%;
        margin: 0;
        border: none;
        border-radius: 4px;
        font-family: sans-serif;
        font-size: 20px;
        letter-spacing: 0.5px;
        background: var(--frc-button-background-color, rgb(230, 230, 230));
        color: var(--frc-button-text-color, black);
      }

      .toggled {
        background: var(--frc-button-toggled-background-color, black);
        color: var(--frc-button-toggled-text-color, white);
        font-weight: bold;
      }
    `,
  ];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-operator-button': OperatorButton;
  }
}
