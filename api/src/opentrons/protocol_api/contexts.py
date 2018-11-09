import asyncio
import logging
from typing import Any, Dict, List, Optional, Union, Tuple

from .labware import Well, Labware, load, load_module, ModuleGeometry
from opentrons import types, hardware_control as hc
import opentrons.config.robot_configs as rc
from opentrons.hardware_control import adapters, modules
from opentrons.helpers import helpers
from . import geometry


MODULE_LOG = logging.getLogger(__name__)

ModuleTypes = Union['TemperatureModuleContext', 'MagneticModuleContext']


class OutOfTipsError(Exception):
    pass


class ProtocolContext:
    """ The Context class is a container for the state of a protocol.

    It encapsulates many of the methods formerly found in the Robot class,
    including labware, instrument, and module loading, as well as core
    functions like pause and resume.

    Unlike the old robot class, it is designed to be ephemeral. The lifetime
    of a particular instance should be about the same as the lifetime of a
    protocol. The only exception is the one stored in
    :py:attr:`.back_compat.robot`, which is provided only for back
    compatibility and should be used less and less as time goes by.
    """

    def __init__(self,
                 loop: asyncio.AbstractEventLoop = None) -> None:
        """ Build a :py:class:`.ProtocolContext`.

        :param loop: An event loop to use. If not specified, this ctor will
                     (eventually) call :py:meth:`asyncio.get_event_loop`.
        """
        self._loop = loop or asyncio.get_event_loop()
        self._deck_layout = geometry.Deck()
        self._instruments: Dict[types.Mount, Optional[InstrumentContext]]\
            = {mount: None for mount in types.Mount}
        self._last_moved_instrument: Optional[types.Mount] = None
        self._location_cache: Optional[types.Location] = None
        self._hardware = self._build_hardware_adapter(self._loop)
        self._log = MODULE_LOG.getChild(self.__class__.__name__)

    def connect(self, hardware: hc.API):
        """ Connect to a running hardware API.

        This can be either a simulator or a full hardware controller.

        Note that there is no true disconnected state for a
        :py:class:`.ProtocolContext`; :py:meth:`disconnect` simply creates
        a new simulator and replaces the current hardware with it.
        """
        self._hardware = self._build_hardware_adapter(self._loop, hardware)
        self._hardware.cache_instruments()

    def disconnect(self):
        """ Disconnect from currently-connected hardware and simulate instead
        """
        self._hardware = self._build_hardware_adapter(self._loop)

    def load_labware(
            self, labware_obj: Labware, location: types.DeckLocation,
            label: str = None, share: bool = False) -> Labware:
        """ Specify the presence of a piece of labware on the OT2 deck.

        This function loads the labware specified by `labware`
        (previously loaded from a configuration file) to the location
        specified by `location`.
        """
        self._deck_layout[location] = labware_obj
        return labware_obj

    def load_labware_by_name(
            self, labware_name: str, location: types.DeckLocation) -> Labware:
        """ A convenience function to specify a piece of labware by name.

        For labware already defined by Opentrons, this is a convient way
        to collapse the two stages of labware initialization (creating
        the labware and adding it to the protocol) into one.

        This function returns the created and initialized labware for use
        later in the protocol.
        """
        labware = load(labware_name,
                       self._deck_layout.position_for(location))
        return self.load_labware(labware, location)

    def load_module(
            self, module_name: str,
            location: types.DeckLocation) -> ModuleTypes:
        for mod in self._hardware.discover_modules():
            if mod.name() == module_name:
                mod_class = {'magdeck': MagneticModuleContext,
                             'tempdeck': TemperatureModuleContext}[module_name]
                break
        else:
            raise KeyError(module_name)
        geometry = load_module(
            module_name, self._deck_layout.position_for(location))
        mod_ctx = mod_class(self,
                            mod,
                            geometry,
                            self._loop)
        self._deck_layout[location] = geometry
        return mod_ctx

    @property
    def loaded_labwares(self) -> Dict[int, Labware]:
        """ Get the labwares that have been loaded into the protocol context.

        The return value is a dict mapping locations to labware, sorted
        in order of the locations.
        """
        return dict(self._deck_layout)

    def load_instrument(
            self,
            instrument_name: str,
            mount: types.Mount,
            tip_racks: List[Labware] = None,
            replace: bool = False) -> 'InstrumentContext':
        """ Load a specific instrument required by the protocol.

        This value will actually be checked when the protocol runs, to
        ensure that the correct instrument is attached in the specified
        location.

        :param str instrument_name: The name of the instrument model, or a
                                    prefix. For instance, 'p10_single' may be
                                    used to request a P10 single regardless of
                                    the version.
        :param types.Mount mount: The mount in which this instrument should be
                                  attached.
        :param tip_racks: A list of tip racks from which to pick tips if
                          :py:meth:`.InstrumentContext.pick_up_tip` is called
                          without arguments.
        :type tip_racks: List[:py:class:`.Labware`]
        :param bool replace: Indicate that the currently-loaded instrument in
                             `mount` (if such an instrument exists) should be
                             replaced by `instrument_name`.
        """
        self._log.info("Trying to load {} on {} mount"
                       .format(instrument_name, mount.name.lower()))
        instr = self._instruments[mount]
        if instr and not replace:
            raise RuntimeError("Instrument already present in {} mount: {}"
                               .format(mount.name.lower(),
                                       instr.name))
        attached = {att_mount: instr.get('name', None)
                    for att_mount, instr
                    in self._hardware.attached_instruments.items()}
        attached[mount] = instrument_name
        self._log.debug("cache instruments expectation: {}"
                        .format(attached))
        self._hardware.cache_instruments(attached)
        # If the cache call didn’t raise, the instrument is attached
        new_instr = InstrumentContext(
            ctx=self,
            hardware=self._hardware,
            mount=mount,
            tip_racks=tip_racks,
            log_parent=self._log)
        self._instruments[mount] = new_instr
        self._log.info("Instrument {} loaded".format(new_instr))
        return new_instr

    @property
    def loaded_instruments(self) -> Dict[str, Optional['InstrumentContext']]:
        """ Get the instruments that have been loaded into the protocol.

        :returns: A dict mapping mount names in lowercase to the instrument
                  in that mount, or `None` if no instrument is present.
        """
        return {mount.name.lower(): instr for mount, instr
                in self._instruments.items()}

    def reset(self):
        """ Reset the state of the context and the hardware.

        For instance, this call will
        - reset all cached knowledge about attached tips
        - unload all labware
        - unload all instruments
        - clear all location and instrument caches

        The only state that will be kept is the position of the robot.
        """
        raise NotImplementedError

    def pause(self):
        """ Pause execution of the protocol until resume is called.

        Note: This function call will not return until the protocol
        is resumed (presumably by a user in the run app).
        """
        raise NotImplementedError

    def resume(self):
        """ Resume a previously-paused protocol. """
        raise NotImplementedError

    def comment(self, msg):
        """ Add a user-readable comment string that will be echoed to the
        Opentrons app. """
        raise NotImplementedError

    @property
    def config(self) -> rc.robot_config:
        """ Get the robot's configuration object.

        :returns .robot_config: The loaded configuration.
        """
        return self._hardware.config

    def update_config(self, **kwargs):
        """ Update values of the robot's configuration.

        `kwargs` should contain keys of the robot's configuration. For instace,
        `update_config(name='Grace Hopper')` would change the name of the robot

        Documentation on keys can be found in the documentation for
        :py:class:`.robot_config`.
        """
        self._hardware.update_config(**kwargs)

    def home(self):
        """ Homes the robot.
        """
        self._log.debug("home")
        self._location_cache = None
        self._hardware.home()

    @property
    def location_cache(self) -> Optional[types.Location]:
        """ The cache used by the robot to determine where it last was.
        """
        return self._location_cache

    @location_cache.setter
    def location_cache(self, loc: Optional[types.Location]):
        self._location_cache = loc

    @property
    def deck(self) -> geometry.Deck:
        """ The object holding the deck layout of the robot.
        """
        return self._deck_layout

    @staticmethod
    def _build_hardware_adapter(
            loop: asyncio.AbstractEventLoop,
            hardware: hc.API = None) -> adapters.SynchronousAdapter:
        if not hardware:
            hardware = hc.API.build_hardware_simulator(loop=loop)
        return adapters.SynchronousAdapter(hardware)


class InstrumentContext:
    """ A context for a specific pipette or instrument.

    This can be used to call methods related to pipettes - moves or
    aspirates or dispenses, or higher-level methods.

    Instances of this class bundle up state and config changes to a
    pipette - for instance, changes to flow rates or trash containers.
    Action methods (like :py:meth:`aspirate` or :py:meth:`distribute`) are
    defined here for convenience.

    In general, this class should not be instantiated directly; rather,
    instances are returned from :py:meth:`ProtcolContext.load_instrument`.
    """

    def __init__(self,
                 ctx: ProtocolContext,
                 hardware: adapters.SynchronousAdapter,
                 mount: types.Mount,
                 log_parent: logging.Logger,
                 tip_racks: List[Labware] = None,
                 **config_kwargs) -> None:
        self._hardware = hardware
        self._ctx = ctx
        self._mount = mount

        self._tip_racks = tip_racks or list()
        for tip_rack in self.tip_racks:
            assert tip_rack.is_tiprack

        self._last_location: Union[Labware, Well, None] = None
        self._log = log_parent.getChild(repr(self))
        self._log.info("attached")
        self._well_bottom_clearance = 0.5

    def aspirate(self,
                 volume: float = None,
                 location: Union[types.Location, Well] = None,
                 rate: float = 1.0) -> 'InstrumentContext':
        """
        Aspirate a volume of liquid (in microliters/uL) using this pipette
        from the specified location

        If only a volume is passed, the pipette will aspirate
        from its current position. If only a location is passed,
        :py:meth:`aspirate` will default to its :py:attr:`max_volume`.

        :param volume: The volume to aspirate, in microliters. If not
                       specified, :py:attr:`max_volume`.
        :type volume: int or float
        :param location: Where to aspirate from. If `location` is a
                         :py:class:`.Well`, the robot will aspirate from
                         :py:attr:`well_bottom_clearance` mm
                         above the bottom of the well. If `location` is a
                         :py:class:`.Location` (i.e. the result of
                         :py:meth:`.Well.top` or :py:meth:`.Well.bottom`), the
                         robot will aspirate from the exact specified location.
                         If unspecified, the robot will aspirate from the
                         current position.
        :param rate: The relative plunger speed for this aspirate. During
                     this aspirate, the speed of the plunger will be
                     `rate` * :py:attr:`aspirate_speed`. If not specified,
                     defaults to 1.0 (speed will not be modified).
        :type rate: float
        :returns: This instance.
        """
        self._log.debug("aspirate {} from {} at {}"
                        .format(volume,
                                location if location else 'current position',
                                rate))
        if isinstance(location, Well):
            point, well = location.bottom()
            self.move_to(
                types.Location(point + types.Point(0, 0,
                                                   self.well_bottom_clearance),
                               well))
        elif isinstance(location, types.Location):
            self.move_to(location)
        elif location is not None:
            raise TypeError(
                'location should be a Well or Location, but it is {}'
                .format(location))
        self._hardware.aspirate(self._mount, volume, rate)
        return self

    def dispense(self,
                 volume: float = None,
                 location: Union[types.Location, Well] = None,
                 rate: float = 1.0) -> 'InstrumentContext':
        """
        Dispense a volume of liquid (in microliters/uL) using this pipette
        into the specified location.

        If only a volume is passed, the pipette will dispense from its current
        position. If only a location is passed, all of the liquid aspirated
        into the pipette will be dispensed (this volume is accessible through
        :py:attr:`current_volume`).

        :param volume: The volume of liquid to dispense, in microliters. If not
                       specified, defaults to :py:attr:`current_volume`.
        :type volume: int or float
        :param location: Where to dispense into. If `location` is a
                         :py:class:`.Well`, the robot will dispense into
                         :py:attr:`well_bottom_clearance` mm
                         above the bottom of the well. If `location` is a
                         :py:class:`.Location` (i.e. the result of
                         :py:meth:`.Well.top` or :py:meth:`.Well.bottom`), the
                         robot will dispense into the exact specified location.
                         If unspecified, the robot will dispense into the
                         current position.
        :param rate: The relative plunger speed for this dispense. During
                     this dispense, the speed of the plunger will be
                     `rate` * :py:attr:`dispense_speed`. If not specified,
                     defaults to 1.0 (speed will not be modified).
        :type rate: float
        :returns: This instance.
        """
        self._log.debug("dispense {} from {} at {}"
                        .format(volume,
                                location if location else 'current position',
                                rate))
        if isinstance(location, Well):
            point, well = location.bottom()
            self.move_to(
                types.Location(point + types.Point(0, 0,
                                                   self.well_bottom_clearance),
                               well))
        elif isinstance(location, types.Location):
            self.move_to(location)
        elif location is not None:
            raise TypeError(
                'location should be a Well or Location, but it is {}'
                .format(location))
        self._hardware.dispense(self._mount, volume, rate)
        return self

    def mix(self,
            repetitions: int = 1,
            volume: float = None,
            location: Union[Well, types.Location] = None,
            rate: float = 1.0) -> 'InstrumentContext':
        """
        Mix a volume of liquid (uL) using this pipette.
        If no location is specified, the pipette will mix from its current
        position. If no Volume is passed, 'mix' will default to its max_volume

        :param repetitions: how many times the pipette should mix (default: 1)
        :param volume: number of microlitres to mix (default: self.max_volume)
        :param location: a Well or a position relative to well
        e.g, `plate.wells('A1').bottom()` (types.Location type)
        :param rate: Set plunger speed for this mix, where
        speed = rate * (aspirate_speed or dispense_speed)
        """
        self._log.debug(
            'mixing {}uL with {} repetitions in {} at rate={}'.format(
                volume, repetitions,
                location if location else 'current position', rate))
        self.aspirate(volume, location, rate)
        for i in range(repetitions -1):
            self.dispense(volume, rate=rate)
            self.aspirate(volume, rate=rate)
        self.dispense(volume, rate=rate)
        return self

    def blow_out(self,
                 location: Union[Well,
                 types.Location] = None) -> 'InstrumentContext':
        """
        Blow liquid out of the tip.

        If called without arguments, blow out into the
        :py:attr:`trash_container`.
        """
        if isinstance(location, Well):
            point, well = location.bottom()
            target = types.Location(point + types.Point(0, 0,
                                                   self.well_bottom_clearance),
                               well)
        elif isinstance(location, types.Location):
            target = location
        elif location is None:
            target = self.trash_container.wells()[0]
        else:
            raise TypeError(
                'location should be a Well or Location, but it is {}'
                .format(location))
        self.move_to(target)
        self._hardware.blow_out(self._mount)
        return self

    def touch_tip(self,
                  location: Well = None,
                  radius: float = 1.0,
                  v_offset: float = -1.0,
                  speed: float = 60.0) -> 'InstrumentContext':
        """
        Touch the Pipette tip to the sides of a well, with the intent of
        removing left-over droplets
        :param location: a Well. If no location is passed, pipette will
        touch_tip at current location
        :param radius: Radius is a floating point describing the percentage of
        a well's radius. When radius=1.0, :any:`touch_tip()` will move to
        100% of the well's radius.
        When radius=0.5, :any:`touch_tip()` will move to 50% of the well's
        radius. Default: 1.0 (100%)
        :param v_offset: The offset in mm from the top of the well to touch tip
        Default: -1.0 mm
        :param speed: The speed for touch tip motion, in mm/s.
        Default: 60.0 mm/s, Max: 80.0 mm/s, Min: 20.0 mm/s
        """
        if not self.hw_pipette['has_tip']:
            raise hc.NoTipAttachedError('Pipette has no tip to touch_tip()')

        if speed > 80.0:
            self._log.warning('Touch tip speed above limit. Setting to 80mm/s')
            speed = 80.0
        elif speed < 20.0:
            self._log.warning('Touch tip speed below min. Setting to 20mm/s')
            speed = 20.0

        if helpers.is_number(location):
            # Deprecated syntax
            self._log.warning("Please use the `v_offset` named parameter")
            v_offset = location
            location = None

        if location is None and self._ctx.location_cache:
            location = self._ctx.location_cache.labware
        if isinstance(location, Well):
            self.move_to(location.top())
        else:
            raise TypeError(
                'location should be a Well, but it is {}'.format(location))

        v_offset = types.Point(0, 0, v_offset)
        well_edges = [
            location._from_center_cartesian(x=radius, y=0, z=1),    #right edge
            location._from_center_cartesian(x=-radius, y=0, z=1),   #left edge
            location._from_center_cartesian(x=0, y=radius, z=1),    #back edge
            location._from_center_cartesian(x=0, y=-radius, z=1)    #front edge
        ]
        # Apply vertical offset to well edges
        well_edges = map(lambda  x: x + v_offset, well_edges)
        for edge in well_edges:
            self._hardware.move_to(self._mount, edge, speed)
        return self

    def air_gap(self,
                volume: float = None,
                height: float = None) -> 'InstrumentContext':
        """
        Pull air into the Pipette current tip.
        :param volume: The amount in uL to aspirate air into the tube.
        (Default will use all remaining volume in tip)
        :param height: The number of millimiters to move above the current Well
        to air-gap aspirate
        (Default will be 5mm above current Well)
        """
        if height is None:
            height = 5
        loc = self._ctx.location_cache.labware
        if isinstance(loc, Well):
            target = loc.top(height)
        else:
            raise RuntimeError('No previous Well cached to perform air gap')
        self._hardware.move_to(target.point)
        self._hardware.aspirate(volume)
        return self

    def return_tip(self) -> 'InstrumentContext':
        if not self.hw_pipette['has_tip']:
            self._log.warning('Pipette has no tip to return')
        # TODO: how do we track last picked up tip?
        raise NotImplementedError

    def pick_up_tip(self, location: types.Location = None,
                    presses: int = 3,
                    increment: float = 1.0) -> 'InstrumentContext':
        """
        Pick up a tip for the pipette to run liquid-handling commands with

        A tip can be manually set by passing a :py:class:`.types.Location`.
        If no location is passed, the Pipette will pick up the next available
        tip in its :py:attr:`InstrumentContext.tip_racks` list

        :param location: The location from which to pick up a tip.
        :type location: :py:class:`.types.Location`
        :param presses: The number of times to lower and then raise the pipette
                        when picking up a tip, to ensure a good seal (0 [zero]
                        will result in the pipette hovering over the tip but
                        not picking it up--generally not desireable, but could
                        be used for dry-run).
        :type presses: int
        :param increment: The additional distance to travel on each successive
                          press (e.g.: if `presses=3` and `increment=1.0`, then
                          the first press will travel down into the tip by
                          3.5mm, the second by 4.5mm, and the third by 5.5mm).
        :type increment: float
        :returns: This instance
        """
        num_channels = \
            self._hardware.attached_instruments[self._mount]['channels']

        def _select_tiprack_from_list(tip_racks) -> Tuple[Labware, Well]:
            try:
                tr = tip_racks[0]
            except IndexError:
                raise OutOfTipsError
            next_tip = tr.next_tip(num_channels)
            if next_tip:
                return tr, next_tip
            else:
                return _select_tiprack_from_list(tip_racks[1:])

        if location and isinstance(location.labware, Labware):
            tiprack = location.labware
            target: Optional[Well] = tiprack.next_tip(num_channels)
        elif location and isinstance(location.labware, Well):
            tiprack = location.labware.parent
            target = location.labware
        else:
            tiprack, target = _select_tiprack_from_list(self.tip_racks)
        if target is None:
            # This is primarily for type checking--should raise earlier
            raise OutOfTipsError

        assert tiprack.is_tiprack

        self.move_to(target.top())

        self._hardware.pick_up_tip(
            self._mount, tiprack.tip_length, presses, increment)
        # Note that the hardware API pick_up_tip action includes homing z after

        tiprack.use_tips(target, num_channels)

        return self

    def drop_tip(self, location: types.Location = None) -> 'InstrumentContext':
        """
        Drop the current tip.

        If a location is specified, drop the tip there, otherwise drop it into
        the fixed trash.

        .. note::
            OT1 required homing the plunger after dropping tips, so the prior
            version of `drop_tip` automatically homed the plunger. This is no
            longer needed in OT2. If you need to home the plunger, use
            :py:meth:`home_plunger`.

        :param location: The location to drop the tip
        :type location: :py:class:`.types.Location` or None

        :returns: This instance
        """
        if location and isinstance(location.labware, Labware):
            target: Well = location.labware.wells()[0]
        elif location and isinstance(location.labware, Well):
            target = location.labware
        else:
            target = self.trash_container.wells()[0]

        self.move_to(target.top())
        self._hardware.drop_tip(self._mount)
        return self

    def home(self) -> 'InstrumentContext':
        """ Home the robot.

        :returns: This instance.
        """
        self._ctx.home()
        return self

    def home_plunger(self) -> 'InstrumentContext':
        """ Home the plunger associated with this mount

        :returns: This instance.
        """
        self._hardware.home_plunger(self.mount)
        return self

    def distribute(self,
                   volume: float,
                   source: Well,
                   dest: List[Well],
                   *args, **kwargs) -> 'InstrumentContext':
        """
        Move a volume of liquid from one source to multiple destinations.
        Refer to transfer() for accepted kwargs
        """
        self._log.debug("Distributing {} from {} to {}"
                        .format(volume, source, dest))
        raise NotImplementedError

    def consolidate(self,
                    volume: float,
                    source: List[Well],
                    dest: Well,

                    *args, **kwargs) -> 'InstrumentContext':
        """
        Move liquid from multiple wells (sources) to a single well(destination)
        Refer to transfer() for accepted kwargs
        """
        self._log.debug("Consolidate {} from {} to {}"
                        .format(volume, source, dest))
        raise NotImplementedError

    def transfer(self,
                 volume: Union[float, Tuple[float], List[float]],
                 source: Union[Well, List[Well]],
                 dest: Union[Well, List[Well]],
                 **kwargs) -> 'InstrumentContext':
        """
        Transfer will move a volume of liquid from a source location(s)
        to a dest location(s). It is a higher-level command, incorporating
        other :any:`Pipette` commands, like :any:`aspirate` and
        :any:`dispense`, designed to make protocol writing easier at the
        cost of specificity.


        :param volume: The amount of volume to remove from each `sources`
        :any:`Placeable` and add to each `targets` :any:`Placeable`.
        If `volume` is a list, each volume will be used for the sources/targets
        at the matching index. If `volumes` is a tuple with two elements,
        like `(20, 100)`, then a list of volumes will be generated with
        a linear gradient between the two volumes in the tuple.

        :param source: Single :any:`Well` or list of :any:`Wells`s, from where
        liquid will be :any:`aspirate`ed.

        :param dest: Single :any:`Well` or list of :any:`Well`s, where
        liquid will be :any:`dispense`ed to.

        :param kwargs:

        new_tip : string
            'never': no tips will be picked up or dropped during the transfer
            'once': (default) a single tip will be used for all commands
            'always': use a new tip for each transfer

        trash : boolean
            If `False` (default behavior) tips will be returned to their
            tip rack. If `True` and a trash container has been attached
            to this `Pipette`, then the tip will be sent to the trash
            container.

        touch_tip : boolean
            If `True`, a :any:`touch_tip` will occur following each
            :any:`aspirate` and :any:`dispense`. If set to `False` (default),
            no :any:`touch_tip` will occur.

        rate: float
            aspirate/dispense flow rate

        blow_out : boolean
            If `True`, a :any:`blow_out` will occur following each
            :any:`dispense`, but only if the pipette has no liquid left in it.
            If set to `False` (default), no :any:`blow_out` will occur.

        mix_before : tuple
            Specify the number of repetitions volume to mix, and a :any:`mix`
            will proceed each :any:`aspirate` during the transfer and dispense.
            The tuple's values is interpreted as (repetitions, volume).

        mix_after : tuple
            Specify the number of repetitions volume to mix, and a :any:`mix`
            will follow each :any:`dispense` during the transfer or
            consolidate. The tuple's values is interpreted as
            (repetitions, volume).

        carryover : boolean
            If `True` (default), any `volumes` that exceed the maximum volume
            of this `Pipette` will be split into multiple smaller volumes.

        repeat : boolean
            (Only applicable to :any:`distribute` and :any:`consolidate`)If
            `True` (default), sequential :any:`aspirate` volumes will be
            combined into one tip for the purpose of saving time. If `False`,
            all volumes will be transferred seperately.

        gradient : lambda
            Function for calculating the curve used for gradient volumes.
            When `volumes` is a tuple of length 2, its values are used
            to create a list of gradient volumes. The default curve for
            this gradient is linear (lambda x: x), however a method can
            be passed with the `gradient` keyword argument to create a
            custom curve.
        """
        self._log.debug("Transfer {} from {} to {}"
                        .format(volume, source, dest))
        tip_options = {
            'once': 1,
            'never': 0,
            'always': float('inf')
        }

        # TODO: implement mode in distribute/ consolidate instead
        kwargs['mode'] = kwargs.get('mode', 'transfer')
        kwargs['touch_tip'] = kwargs.get('touch_tip', False)
        kwargs['new_tip'] = kwargs.get('new_tip', 'once')
        kwargs['air_gap'] = kwargs.get('air_gap', 0)
        kwargs['carryover'] = kwargs.get('carryover', True)
        kwargs['rate'] = kwargs.get('rate', 1)
        kwargs['mix_before'] = kwargs.get('mix_before', (0, 0))
        kwargs['mix_after'] = kwargs.get('mix_after', (0,0))

        tips = tip_options[kwargs['new_tip']]
        if tips is None:
            raise ValueError('Unknown "new_tip" option: {}'.
                             format(kwargs['new_tip']))

        plan = self._create_tranfer_plan(volume, source, dest, **kwargs)
        raise NotImplementedError

    def _create_tranfer_plan(self, vol, src, dest, **kwargs):
        """
        Transfer plan is a list of dicts consisting of 'aspirate' and
        'dispense' locations and volumes in the following way:
        [{'aspirate': {'location': <source1>, 'volume': <float(volume1)>},
          'dispense': {'location': <target1>, 'volume': <float(volume1)}>},
         {....},
         {....},
         {'aspirate': {'location': <sourceN>, 'volume': <float(volumeN)>},
          'dispense': {'location': <targetN>, 'volume': <float(volumeN)>}},...]

        The plan accounts for pipette volume limits by dividing volumes that
        exceed max_volume into smaller amounts.
        The plan tries to achieve the requested transfer in the minimum number
        of steps by combining several aspirates (in case of distribute) or
        dispenses (in case of consolidate)
        """
        xfer_plan = []
        # CASES:
        # i. if using multi-channel pipette,
        # and the source or target is a row/column of Wells (i.e list of Wells)
        # then avoid iterating through its Wells.
        # ii. if using single channel pipettes, flatten a multi-dimensional
        # list of Wells into a 1 dimensional list of Wells
        if self.hw_pipette['channels'] > 1:
            src, dest = self._multichannel_transfer(src, dest)
        else:
            if isinstance(src, List) and isinstance(src[0], List):
                # Source is a List[List[Well]]
                # TODO: Check if this is a valid arg
                src = [well for series in src for well in series]
            if isinstance(dest, List) and isinstance(dest[0], List):
                # Dest is a List[List[Well]]
                # TODO: Check if this is a valid arg
                dest = [well for series in dest for well in series]

        # Create a list of sources and targets of equal length
        # So, if src = [S1, S2] & dest = [D1, D2, D3, D4],
        # s = [S1, S1, S2, S2] & d = [D1, D2, D3, D4]
        s, t = helpers._create_source_target_lists(src, dest, **kwargs)

        total_xfers = len(t)
        # Create a list of volumes to be used per transfer: either a volume
        # gradient or the user-specified list or a list of constant vol
        # for each transfer
        v = helpers._create_volume_list(vol, total_xfers, **kwargs)

        for i in range(total_xfers):
            xfer_plan.append({
                'aspirate': {'location': s[i], 'volume': v[i]},
                'dispense': {'location': t[i], 'volume': v[i]}
            })

        max_vol = self.max_volume - kwargs['air_gap']

        if kwargs['carryover']:
            xfer_plan = helpers._expand_for_carryover(max_vol,
                                                      xfer_plan, **kwargs)
        if kwargs['mode'] == 'distribute':
            # combine target volumes into single aspirate
            xfer_plan = helpers._compress_for_distribute(max_vol,
                                                         xfer_plan, **kwargs)
        elif kwargs['mode'] == 'consolidate':
            # combine target volumes into multiple aspirates
            xfer_plan = helpers._compress_for_consolidate(max_vol,
                                                          xfer_plan, **kwargs)
        return xfer_plan

    def _run_transfer_plan(self, tips, plan, **kwargs):
        air_gap = kwargs['air_gap']
        touch_tip = kwargs['touch_tip']

        total_xfers = len(plan)
        for i, step in enumerate(plan):
            aspirate = step.get('aspirate')
            dispense = step.get('dispense')

            if aspirate:
                self._add_tip_during_transfer(tips)
                if self.current_volume == 0:
                    self._mix_during_transfer(kwargs['mix_before'],
                                              aspirate['location'], **kwargs)
                self.aspirate(aspirate['volume'],
                              aspirate['location'], rate=kwargs['rate'])
                if air_gap:
                    self.air_gap(air_gap)

    def _add_tip_during_transfer(self, tips):
        """
        Performs a :any:`pick_up_tip` when running a :any:`transfer`,
        :any:`distribute`, or :any:`consolidate`.
        """
        # 'tips' is reduced after drop_tip()
        if tips > 0:
            try:
                self.pick_up_tip()
            except:
                raise

    def _mix_during_transfer(self, mix: tuple[int, int], loc: Well, **kwargs):
        raise NotImplementedError

    def _multichannel_transfer(self, s, d):
        # Helper function for multi-channel use-case
        assert isinstance(s, Well) or \
               (isinstance(s, List) and not isinstance(s[0], List)),\
               'Source should be a Well or List[Well] but is {}'.format(s)
        assert isinstance(d, Well) or \
            (isinstance(d, List) and not isinstance(d[0], List)),\
            'Target should be a Well or List[Well] but is {}'.format(d)
        raise NotImplementedError

    def move_to(self, location: types.Location) -> 'InstrumentContext':
        """ Move the instrument.

        :param location: The location to move to.
        :type location: :py:class:`.types.Location`
        """
        if self._ctx.location_cache:
            from_lw = self._ctx.location_cache.labware
        else:
            from_lw = None
        from_loc = types.Location(self._hardware.gantry_position(self._mount),
                                  from_lw)
        moves = geometry.plan_moves(from_loc, location, self._ctx.deck)
        self._log.debug("move {}->{}: {}"
                        .format(from_loc, location, moves))
        try:
            for move in moves:
                self._hardware.move_to(self._mount, move)
        except Exception:
            self._ctx.location_cache = None
            raise
        else:
            self._ctx.location_cache = location
        return self

    @property
    def mount(self) -> str:
        return self._mount.name.lower()

    @property
    def speeds(self) -> Dict[str, float]:
        """ The speeds (in mm/s) configured for the pipette, as a dict.

        The keys will be 'aspirate' and 'dispense' (e.g. the keys of
        :py:class:`MODE`)

        :note: This property is equivalent to :py:attr:`speeds`; the only
        difference is the units in which this property is specified.
        """
        raise NotImplementedError

    @speeds.setter
    def speeds(self, new_speeds: Dict[str, float]) -> None:
        """ Update the speeds (in mm/s) set for the pipette.

        :param new_speeds: A dict containing at least one of 'aspirate'
        and 'dispense',  mapping to new speeds in mm/s.
        """
        raise NotImplementedError

    @property
    def flow_rate(self) -> Dict[str, float]:
        """ The speeds (in uL/s) configured for the pipette, as a dict.

        The  keys will be 'aspirate' and 'dispense'.

        :note: This property is equivalent to :py:attr:`speeds`; the only
        difference is the units in which this property is specified.
        """
        raise NotImplementedError

    @flow_rate.setter
    def flow_rate(self, new_flow_rate: Dict[str, float]) -> None:
        """ Update the speeds (in uL/s) for the pipette.

        :param new_flow_rates: A dict containing at least one of 'aspirate
        and 'dispense', mapping to new speeds in uL/s.
        """
        raise NotImplementedError

    @property
    def pick_up_current(self) -> float:
        """
        The current (amperes) the pipette mount's motor will use
        while picking up a tip. Specified in amps.
        """
        raise NotImplementedError

    @pick_up_current.setter
    def pick_up_current(self, amps: float):
        """ Set the current used when picking up a tip.

        :param amps: The current, in amperes. Acceptable values: (0.0, 2.0)
        """
        raise NotImplementedError

    @property
    def type(self) -> str:
        """ One of `'single'` or `'multi'`.
        """
        model = self.name
        if 'single' in model:
            return 'single'
        elif 'multi' in model:
            return 'multi'
        else:
            raise RuntimeError("Bad pipette model name: {}".format(model))

    @property
    def tip_racks(self) -> List[Labware]:
        """
        The tip racks that have been linked to this pipette.

        This is the property used to determine which tips to pick up next when
        calling :py:meth:`pick_up_tip` without arguments.
        """
        return self._tip_racks

    @tip_racks.setter
    def tip_racks(self, racks: List[Labware]):
        self._tip_racks = racks

    @property
    def trash_container(self) -> Labware:
        """ The trash container associated with this pipette.

        This is the property used to determine where to drop tips and blow out
        liquids when calling :py:meth:`drop_tip` or :py:meth:`blow_out` without
        arguments.
        """
        raise NotImplementedError

    @trash_container.setter
    def trash_container(self, trash: Labware):
        raise NotImplementedError

    @property
    def name(self) -> str:
        """
        The model string for the pipette.
        """
        return self.hw_pipette['name']

    @property
    def max_volume(self) -> float:
        """
        The maximum volume, in microliters, this pipette can hold.
        """
        return self.hw_pipette['max_volume']

    @property
    def current_volume(self) -> float:
        """
        The current amount of liquid, in microliters, held in the pipette.
        """
        return self.hw_pipette['current_volume']

    @property
    def hw_pipette(self) -> Dict[str, Any]:
        """ View the information returned by the hardware API directly.

        :raises: a :py:class:`.types.PipetteNotAttachedError` if the pipette is
                 no longer attached (should not happen).
        """
        pipette = self._hardware.attached_instruments[self._mount]
        if pipette is None:
            raise types.PipetteNotAttachedError
        return pipette

    @property
    def well_bottom_clearance(self) -> float:
        """ The distance above the bottom of a well to aspirate or dispense.

        When :py:meth:`aspirate` or :py:meth:`dispense` is given a
        :py:class:`.Well` rather than a full :py:class:`.Location`, the robot
        will move this distance above the bottom of the well to aspirate or
        dispense.
        """
        return self._well_bottom_clearance

    @well_bottom_clearance.setter
    def well_bottom_clearance(self, clearance: float):
        assert clearance >= 0
        self._well_bottom_clearance = clearance

    def __repr__(self):
        return '<{}: {} in {}>'.format(self.__class__.__name__,
                                       self.hw_pipette['name'],
                                       self._mount.name)

    def __str__(self):
        return '{} on {} mount'.format(self.hw_pipette['display_name'],
                                       self._mount.name.lower())


class ModuleContext:
    """ An object representing a connected module. """

    def __init__(self, ctx: ProtocolContext, geometry: ModuleGeometry) -> None:
        """ Build the ModuleContext.

        This usually should not be instantiated directly; instead, modules
        should be loaded using :py:meth:`ProtocolContext.load_module`.

        :param ctx: The parent context for the module
        :param geometry: The :py:class:`.ModuleGeometry` for the module
        """
        self._geometry = geometry
        self._ctx = ctx

    def load_labware(self, labware: Labware) -> Labware:
        """ Specify the presence of a piece of labware on the module.

        :param labware: The labware object. This object should be already
                        initialized and its parent should be set to this
                        module's geometry. To initialize and load a labware
                        onto the module in one step, see
                        :py:meth:`load_labware_by_name`.
        :returns: The properly-linked labware object
        """
        self._geometry.add_labware(labware)
        self._ctx.deck.recalculate_high_z()
        return labware

    def load_labware_by_name(self, name: str) -> Labware:
        """ Specify the presence of a piece of labware on the module.

        :param name: The name of the labware object.
        :returns: The initialized and loaded labware object.
        """
        lw = load(name, self._geometry.location)
        return self.load_labware(lw)

    @property
    def labware(self) -> Optional[Labware]:
        """ The labware (if any) present on this module. """
        return self._geometry.labware

    def __repr__(self):
        return "{} at {} lw {}".format(self.__class__.__name__,
                                       self._geometry,
                                       self.labware)


class TemperatureModuleContext(ModuleContext):
    """ An object representing a connected Temperature Module.

    It should not be instantiated directly; instead, it should be
    created through :py:meth:`.ProtocolContext.load_module`.
    """
    def __init__(self, ctx: ProtocolContext,
                 hw_module: modules.tempdeck.TempDeck,
                 geometry: ModuleGeometry,
                 loop: asyncio.AbstractEventLoop):
        self._module = hw_module
        self._loop = loop
        super().__init__(ctx, geometry)

    def set_temperature(self, celsius: float):
        """ Set the target temperature, in C.

        Must be between 4 and 95C based on Opentrons QA.

        :param celsius: The target temperature, in C
        """
        return self._module.set_temperature(celsius)

    def deactivate(self):
        """ Stop heating (or cooling) and turn off the fan.
        """
        return self._module.disengage()

    def wait_for_temp(self):
        """ Block until the module reaches its setpoint.
        """
        self._loop.run_until_complete(self._module.wait_for_temp())

    @property
    def temperature(self):
        """ Current temperature in C"""
        return self._module.temperature

    @property
    def target(self):
        """ Current target temperature in C"""
        return self._module.target


class MagneticModuleContext(ModuleContext):
    """ An object representing a connected Temperature Module.

    It should not be instantiated directly; instead, it should be
    created through :py:meth:`.ProtocolContext.load_module`.
    """
    def __init__(self,
                 ctx: ProtocolContext,
                 hw_module: modules.magdeck.MagDeck,
                 geometry: ModuleGeometry,
                 loop: asyncio.AbstractEventLoop) -> None:
        self._module = hw_module
        self._loop = loop
        super().__init__(ctx, geometry)

    def calibrate(self):
        """ Calibrate the Magnetic Module.

        The calibration is used to establish the position of the lawbare on
        top of the magnetic module.
        """
        self._module.calibrate()

    def load_labware(self, labware: Labware) -> Labware:
        """
        Load labware onto a Magnetic Module, checking if it is compatible
        """
        if labware.magdeck_engage_height is None:
            MODULE_LOG.warning(
                "This labware ({}) is not explicitly compatible with the"
                " Magnetic Module. You will have to specify a height when"
                " calling engage().")
        return super().load_labware(labware)

    def engage(self, height: float = None, offset: float = None):
        """ Raise the Magnetic Module's magnets.

        The destination of the magnets can be specified in several different
        ways, based on internally stored default heights for labware:

           - If neither `height` nor `offset` is specified, the magnets will
             raise to a reasonable default height based on the specified
             labware.
           - If `height` is specified, it should be a distance in mm from the
             home position of the magnets.
           - If `offset` is specified, it should be an offset in mm from the
             default position. A positive number moves the magnets higher and
             a negative number moves the magnets lower.

        Only certain labwares have defined engage heights for the Magnetic
        Module. If a labware that does not have a defined engage height is
        loaded on the Magnetic Module (or if no labware is loaded), then
        `height` must be specified.

        :param height: The height to raise the magnets to, in mm from home.
        :param offset: An offset relative to the default height for the labware
                       in mm
        """
        if height:
            dist = height
        elif self.labware and self.labware.magdeck_engage_height is not None:
            dist = self.labware.magdeck_engage_height
            if offset:
                dist += offset
        else:
            raise ValueError(
                "Currently loaded labware {} does not have a known engage "
                "height; please specify explicitly with the height param"
                .format(self.labware))
        self._module.engage(dist)

    def disengage(self):
        """ Lower the magnets back into the Magnetic Module.
        """
        self._module.disengage()

    @property
    def status(self):
        """ The status of the module. either 'engaged' or 'disengaged' """
        return self._module.status
