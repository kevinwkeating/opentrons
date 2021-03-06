// @flow

import type {PipetteData} from '../step-generation'
import type {PipetteFields} from '../load-file'

export type PipettesByMount = {
  left?: ?PipetteData,
  right?: ?PipetteData,
}

export type UpdatePipettesAction = {
  type: 'UPDATE_PIPETTES',
  payload: PipettesByMount,
}

export type EditPipettesFields = {
  left: PipetteFields,
  right: PipetteFields,
}

export type FormattedPipette = {
  id: $PropertyType<PipetteData, 'id'>,
  mount: $PropertyType<PipetteData, 'mount'>,
  model: $PropertyType<PipetteData, 'model'>,
  maxVolume: $PropertyType<PipetteData, 'maxVolume'>,
  channels: $PropertyType<PipetteData, 'channels'>,
  description: string,
  isDisabled: boolean,
  tiprackModel: ?string,
  tiprack: {model: $PropertyType<PipetteData, 'tiprackModel'>},
}
