// @flow
import type {StepFieldName} from '../fieldLevel'

// Update Form input (onChange on inputs)
export type ChangeFormPayload = {
  // TODO Ian 2018-05-04 use StepType + FormData type to properly type this payload.
  // Accessor strings and values depend on StepType
  stepType?: string,
  update: {
    [StepFieldName]: ?mixed, // string | boolean | Array<string> | null,
  },
}

export type ChangeSavedFormPayload = {
  stepId?: string,
  update: {
    [StepFieldName]: ?mixed, // string | boolean | Array<string> | null,
  },
}
