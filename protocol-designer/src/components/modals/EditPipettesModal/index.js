// @flow
import * as React from 'react'
import cx from 'classnames'
import {connect} from 'react-redux'

import {
  AlertModal,
  DropdownField,
  FormGroup,
  type Mount,
} from '@opentrons/components'
import startCase from 'lodash/startCase'
import isEmpty from 'lodash/isEmpty'
import isEqual from 'lodash/isEqual'

import i18n from '../../../localization'
import type {BaseState, ThunkDispatch} from '../../../types'
import {pipetteOptions} from '../../../pipettes/pipetteData'
import type {PipetteFields} from '../../../load-file'
import {
  thunks as pipetteThunks,
  selectors as pipetteSelectors,
  type EditPipettesFields,
  type FormattedPipette,
} from '../../../pipettes'

import PipetteDiagram from '../NewFileModal/PipetteDiagram'
import TiprackDiagram from '../NewFileModal/TiprackDiagram'
import formStyles from '../../forms.css'
import styles from './EditPipettesModal.css'
import modalStyles from '../modal.css'
import StepChangesWarningModal from './StepChangesWarningModal'

type State = EditPipettesFields & {isWarningModalOpen: boolean}

type OP = {closeModal: () => void}
type SP = {
  initialLeft: ?FormattedPipette,
  initialRight: ?FormattedPipette,
}
type DP = {
  updatePipettes: (EditPipettesFields) => mixed,
}

type Props = SP & DP & OP

const pipetteOptionsWithNone = [
  {name: 'None', value: ''},
  ...pipetteOptions,
]

// TODO: Ian 2018-06-22 get this programatically from shared-data labware defs
// and exclude options that are incompatible with pipette
// and also auto-select tiprack if there's only one compatible tiprack for a pipette
const tiprackOptions = [
  {name: '10 μL', value: 'tiprack-10ul'},
  {name: '200 μL', value: 'tiprack-200ul'},
  {name: '300 μL', value: 'opentrons-tiprack-300ul'},
  {name: '1000 μL', value: 'tiprack-1000ul'},
]

const DEFAULT_SELECTION = {pipetteModel: '', tiprackModel: null}

const pipetteDataToFormState = (pipetteData) => ({
  pipetteModel: (pipetteData && pipetteData.model) ? pipetteData.model : '',
  tiprackModel: (pipetteData && pipetteData.tiprack && pipetteData.tiprack.model) ? pipetteData.tiprack.model : null,
})

class EditPipettesModal extends React.Component<Props, State> {
  constructor (props) {
    super(props)
    const {initialLeft, initialRight} = props
    this.state = {
      left: initialLeft ? pipetteDataToFormState(initialLeft) : DEFAULT_SELECTION,
      right: initialRight ? pipetteDataToFormState(initialRight) : DEFAULT_SELECTION,
      isWarningModalOpen: false,
    }
  }

  makeHandleMountChange = (mount: Mount, fieldName: $Keys<PipetteFields>) => (e: SyntheticInputEvent<*>) => {
    const value: string = e.target.value
    let nextMountState = {[fieldName]: value}
    if (fieldName === 'pipetteModel') nextMountState = {...nextMountState, tiprackModel: null}
    this.setState({[mount]: {...this.state[mount], ...nextMountState}})
  }

  handleSubmit = () => {
    const {initialLeft, initialRight} = this.props
    const {left, right} = this.state
    const initialLeftFormData = pipetteDataToFormState(initialLeft)
    const initialRightFormData = pipetteDataToFormState(initialRight)
    const leftChanged = !isEqual(initialLeft, left)
    const rightChanged = !isEqual(initialRight, right)
    if ((leftChanged && !isEmpty(initialLeftFormData.pipetteModel)) || (rightChanged && !isEmpty(initialRightFormData.pipetteModel))) {
      this.setState({isWarningModalOpen: true})
    } else {
      this.savePipettes()
    }
  }

  savePipettes = () => {
    const {left, right} = this.state
    this.props.updatePipettes({left, right})
    this.props.closeModal()
  }

  handleCancel = () => {
    this.props.closeModal()
  }

  render () {
    const {left, right} = this.state

    const pipetteSelectionIsValid = (
      // at least one must not be none (empty string)
      (left.pipetteModel || right.pipetteModel)
    )

    // if pipette selected, corresponding tiprack type also selected
    const tiprackSelectionIsValid = (
      (left.pipetteModel ? Boolean(left.tiprackModel) : true) &&
      (right.pipetteModel ? Boolean(right.tiprackModel) : true)
    )

    const canSubmit = pipetteSelectionIsValid && tiprackSelectionIsValid

    return (
      <React.Fragment>
        <AlertModal
          className={cx(modalStyles.modal, styles.new_file_modal)}
          contentsClassName={styles.modal_contents}
          buttons={[
            {onClick: this.handleCancel, children: 'Cancel', tabIndex: 7},
            {onClick: this.handleSubmit, disabled: !canSubmit, children: 'Save', tabIndex: 6},
          ]}>
          <form onSubmit={() => { canSubmit && this.handleSubmit() }}>
            <h2>{i18n.t('modal.edit_pipettes.title')}</h2>
            <p className={styles.edit_pipettes_description}>{i18n.t('modal.edit_pipettes.body')}</p>

            <div className={styles.mount_fields_row}>
              <div className={styles.mount_column}>
                <FormGroup key="leftPipetteModel" label="Left Pipette" className={formStyles.stacked_row}>
                  <DropdownField
                    tabIndex={2}
                    options={pipetteOptionsWithNone}
                    value={this.state.left.pipetteModel}
                    onChange={this.makeHandleMountChange('left', 'pipetteModel')} />
                </FormGroup>
                <FormGroup
                  disabled={isEmpty(this.state.left.pipetteModel)}
                  key={'leftTiprackModel'}
                  label={`${startCase('left')} Tiprack*`}
                  className={formStyles.stacked_row}>
                  <DropdownField
                    tabIndex={3}
                    disabled={isEmpty(this.state.left.pipetteModel)}
                    options={tiprackOptions}
                    value={this.state.left.tiprackModel}
                    onChange={this.makeHandleMountChange('left', 'tiprackModel')} />
                </FormGroup>
              </div>
              <div className={styles.mount_column}>
                <FormGroup key="rightPipetteModel" label="Right Pipette" className={formStyles.stacked_row}>
                  <DropdownField
                    tabIndex={4}
                    options={pipetteOptionsWithNone}
                    value={this.state.right.pipetteModel}
                    onChange={this.makeHandleMountChange('right', 'pipetteModel')} />
                </FormGroup>
                <FormGroup
                  disabled={isEmpty(this.state.right.pipetteModel)}
                  key={'rightTiprackModel'}
                  label={`${startCase('right')} Tiprack*`}
                  className={formStyles.stacked_row}>
                  <DropdownField
                    tabIndex={5}
                    disabled={isEmpty(this.state.right.pipetteModel)}
                    options={tiprackOptions}
                    value={this.state.right.tiprackModel}
                    onChange={this.makeHandleMountChange('right', 'tiprackModel')} />
                </FormGroup>
              </div>
            </div>

            <div className={styles.diagrams}>
              <TiprackDiagram containerType={this.state.left.tiprackModel} />
              <PipetteDiagram
                leftPipette={this.state.left.pipetteModel}
                rightPipette={this.state.right.pipetteModel}
              />
              <TiprackDiagram containerType={this.state.right.tiprackModel} />
            </div>
          </form>
        </AlertModal>
        {
          this.state.isWarningModalOpen &&
          <StepChangesWarningModal onCancel={this.handleCancel} onConfirm={this.savePipettes} />
        }
      </React.Fragment>
    )
  }
}

const mapSTP = (state: BaseState): SP => {
  const pipetteData = pipetteSelectors.getPipettesForEditPipettes(state)
  return {
    initialLeft: pipetteData.find(i => i.mount === 'left'),
    initialRight: pipetteData.find(i => i.mount === 'right'),
  }
}

const mapDTP = (dispatch: ThunkDispatch<*>): DP => ({
  updatePipettes: (fields: EditPipettesFields) => {
    dispatch(pipetteThunks.editPipettes(fields))
  },
})

export default connect(mapSTP, mapDTP)(EditPipettesModal)
