import { NetworkTables } from './network-tables';

export function ensureSendableChooserInitialized(nt: NetworkTables, key: string) {
  // TODO: handle restarts of robot after initial page load
  let selectedAutoMode = '';
  nt.addKeyListener(
    key,
    (_, value: { default: string; selected: string }) => {
      if (value && (selectedAutoMode ?? '') === '') {
        if (value && Object.hasOwn(value, 'selected')) {
          selectedAutoMode = value.selected;
        } else if (Object.hasOwn(value, 'default')) {
          selectedAutoMode = value.default;
        }
      }
      if (value && (!Object.hasOwn(value, 'selected') || value.selected === '') && Object.hasOwn(value, 'default')) {
        nt.setValue(`${key}/selected`, (selectedAutoMode ?? '') !== '' ? selectedAutoMode : value.default);
      }
    },
    true
  );
}
