import { LitElement, css, html } from 'lit';
import { customElement, property } from 'lit/decorators.js';
import { ClassInfo, classMap } from 'lit/directives/class-map.js';
import { globalStylesCss } from '../styles/styles';

@customElement('team1701-match-timer')
export class MatchTimer extends LitElement {
  @property({ type: Number }) timer = 0;

  render() {
    const roundedTimer = Math.floor(this.timer);
    return html`<div class="flex flex-column align-center justify-center">
      <div class="text-white text-center text-6xl border-4 border-solid rounded-lg w-44 box-border p-4 ${classMap(this.backgroundColor())}">
        ${roundedTimer}
      </div>
    </div>`;
  }

  backgroundColor(): ClassInfo {
    if (this.timer > 45) {
      return { 'bg-blue-700': true };
    } else if (this.timer > 20) {
      return { 'bg-yellow-500': true };
    } else {
      return { 'bg-red-900': true };
    }
  }

  static styles = [globalStylesCss, css``];
}

declare global {
  interface HTMLElementTagNameMap {
    'team1701-match-timer': MatchTimer;
  }
}
