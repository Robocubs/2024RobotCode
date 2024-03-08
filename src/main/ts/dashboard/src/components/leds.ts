import { LitElement, TemplateResult, css, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { map } from 'lit/directives/map.js';
import { globalStylesCss } from '../styles/styles';

@customElement('team1701-leds')
export class Leds extends LitElement {
  @property({ type: Array }) colors: string[] = [];

  ledColors(): string[] {
    const cols = 9;
    const rows = this.colors.length / cols;
    const correctedColors = [];
    for (let row = 0; row < rows; row++) {
      for (let col = 0; col < cols; col++) {
        correctedColors.push(row % 2 === 1 ? this.colors[row * cols + col] : this.colors[row * cols + (cols - col - 1)]);
      }
    }

    return correctedColors;
  }

  render(): TemplateResult {
    return html`<div class="flex flex-col gap-4 text-white w-full h-full">
      <div class="grid grid-cols-9 gap-4 w-full">
        ${map(this.ledColors(), (color) => html`<div class="aspect-square" style="background:${color};"></div>`)}
      </div>
    </div>`;
  }

  static styles = [
    globalStylesCss,
    css`
      div {
        margin: 0;
        border: none;
        border-radius: 4px;
      }
    `,
  ];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-leds': Leds;
  }
}
