// @flow
import isArray from 'lodash/isArray'

/*******************
** Error Messages **
********************/

// TODO: reconcile difference between returning error string and key

export type FieldError =
  | 'REQUIRED'
  | 'UNDER_WELL_MINIMUM'
  | 'NON_ZERO'

const FIELD_ERRORS: {[FieldError]: string} = {
  REQUIRED: 'This field is required',
  UNDER_WELL_MINIMUM: 'or more wells are required',
  NON_ZERO: 'Must be greater than zero',
}

// TODO: test these
/*******************
** Error Checkers **
********************/
type ErrorChecker = (value: mixed) => ?string

export const requiredField = (value: mixed): ?string => !value ? FIELD_ERRORS.REQUIRED : null
export const nonZero = (value: mixed) => (value && Number(value) === 0) ? FIELD_ERRORS.NON_ZERO : null
export const minimumWellCount = (minimum: number): ErrorChecker => (wells: mixed): ?string => (
  (isArray(wells) && (wells.length < minimum)) ? `${minimum} ${FIELD_ERRORS.UNDER_WELL_MINIMUM}` : null
)

/*******************
**     Helpers    **
********************/

export const composeErrors = (...errorCheckers: Array<ErrorChecker>) => (value: mixed): Array<string> => (
  errorCheckers.reduce((accumulatedErrors, errorChecker) => {
    const possibleError = errorChecker(value)
    return possibleError ? [...accumulatedErrors, possibleError] : accumulatedErrors
  }, [])
)
