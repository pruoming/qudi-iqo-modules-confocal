# -*- coding: utf-8 -*-
"""
JSON exchange IO for qudi pulse objects — the qudi-independent pulse editor.

Contract: Qudi_AI/tools/pulse_editor/pulse_sequence_format.md, format_version 1 (consultant
T1+T2, owner-approved 2026-08-10) and **format_version 2** (T16, owner-approved 2026-08-13:
optional display "label" strings on elements and blocks). This module is deliberately THIN:
it moves data between JSON and the dict representation that the stock pulse objects already
define (get_dict_representation / *_from_dict) and hands persistence to the stock
SequenceGeneratorLogic save path (save_block / save_ensemble / save_sequence — which persist
as pickle; JSON is the exchange layer, not the storage layer). NO serialization semantics are
added here beyond JSON<->dict plus loud validation.

format_version 2 — optional display labels:
  A v2 element or block MAY carry an OPTIONAL "label": str. Labels are a DISPLAY namespace
  only: they are STRIPPED before the stock *_from_dict constructors run, so qudi objects
  never see them and NO attribute is added to any stock class. Every v1 DATA rule is
  unchanged (an element still has exactly the five DATA keys; a block still has exactly
  name + element_list). v1 stays STRICT: a v1 file carrying a label is rejected.

  **ROUND-TRIP THROUGH QUDI IS LOSSY FOR LABELS.** Because qudi pulse objects cannot carry a
  label, exporting them produces a v2 file with NO labels. Labels survive only tool-side
  (editor -> JSON -> editor); importing a labeled file into qudi and re-exporting DROPS every
  label. This is by design — labels are editor annotations, not qudi state.

Import hard rules (format doc §7): reject format_version > 2; validate sampling-function
names against the INSTALLED SamplingFunctions set before construction (unknown name = clear
error naming the offender and the legal set — never a guess); element DATA keys must be
exactly the five contract keys (plus an optional v2 label); block_list arrays are normalized
back to (str, int) tuples; a referenced block/ensemble must be in the file or already saved
on the importing side.

Export hard rules: emits format_version 2; qudi objects carry no labels so none are written
(see the lossy caveat above). allow_nan=False (non-finite numbers fail loudly); numpy SCALARS
are coerced to plain Python numbers; numpy ARRAYS are rejected with a clear message (empty the
transient dicts or convert to lists first).

Usage (qudi manager console, mock config):
    from qudi.logic.pulsed.pulsed_json_io import export_to_json, import_from_json
    sgl = sequence_generator_logic   # the running SequenceGeneratorLogic instance
    export_to_json(file_path='C:/.../assets.pulse.json',
                   blocks=[sgl.get_block('p0_rabi_blk')],
                   ensembles=[sgl.get_ensemble('p0_incr')])
    created = import_from_json('C:/.../assets.pulse.json', sgl, save=True)

Copyright (c) 2026, the qudi developers. qudi is free software licensed under LGPL v3 —
see the qudi-iqo-modules LICENSE files.
"""

__all__ = ['FORMAT_VERSION', 'PulseJsonError', 'export_to_json', 'import_from_json',
           'play_ready']

import copy
import json
import numbers
import os
import time

import numpy as np

from qudi.logic.pulsed.pulse_objects import (PulseBlock, PulseBlockElement,
                                             PulseBlockEnsemble, PulseSequence)
from qudi.logic.pulsed.sampling_functions import SamplingFunctions

FORMAT_VERSION = 2

_ELEMENT_KEYS = {'init_length_s', 'increment_s', 'laser_on', 'digital_high', 'pulse_function'}
_CONTAINER_KEYS = {'format_version', 'blocks', 'ensembles', 'sequences'}


class PulseJsonError(ValueError):
    """Raised on any violation of the pulse-sequence JSON exchange contract (v1/v2)."""
    pass


def _check_label(obj, where):
    """v2 optional display label must be a plain string when present. Returns nothing;
    raises on a non-string label."""
    if 'label' in obj and not isinstance(obj['label'], str):
        raise PulseJsonError('{0}: "label" must be a string (v2 display label); got {1}.'
                             ''.format(where, type(obj['label']).__name__))


def _strip_labels(blk_dict):
    """Remove the v2 display 'label' from a block dict and each of its element dicts, IN
    PLACE, so the stock *_from_dict constructors (PulseBlock(**d) / PulseBlockElement(**d))
    never receive it — qudi objects carry no label attribute."""
    blk_dict.pop('label', None)
    for el in blk_dict.get('element_list', []):
        if isinstance(el, dict):
            el.pop('label', None)
    return blk_dict


# ---------------------------------------------------------------------------- export helpers

def _json_default(obj):
    """Coerce numpy scalars; reject everything else loudly (format doc §2 rule 5)."""
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, np.ndarray):
        raise PulseJsonError(
            'numpy arrays are not representable in pulse-JSON (found one while exporting). '
            'Empty the transient dicts (sampling_information / measurement_information / '
            'generation_method_parameters) or convert the value to a plain list first.')
    raise PulseJsonError('Object of type {0} is not representable in pulse-JSON.'
                         ''.format(type(obj).__name__))


def export_to_json(file_path=None, blocks=None, ensembles=None, sequences=None):
    """Serialize pulse objects to a format_version-2 JSON container.

    qudi objects carry no display label, so the emitted file has NONE — round-tripping a
    labeled file through qudi drops every label (see the module docstring's lossy caveat).
    Labels are a tool-side (editor) concern.

    @param str file_path: optional path to write; the JSON string is always returned
    @param iterable blocks: PulseBlock instances
    @param iterable ensembles: PulseBlockEnsemble instances
    @param iterable sequences: PulseSequence instances
    @return str: the JSON document
    """
    container = {'format_version': FORMAT_VERSION, 'blocks': [], 'ensembles': [],
                 'sequences': []}
    for blk in (blocks or ()):
        if not isinstance(blk, PulseBlock):
            raise PulseJsonError('blocks entries must be PulseBlock instances, got {0}'
                                 ''.format(type(blk).__name__))
        container['blocks'].append(blk.get_dict_representation())
    for ens in (ensembles or ()):
        if not isinstance(ens, PulseBlockEnsemble):
            raise PulseJsonError('ensembles entries must be PulseBlockEnsemble instances, '
                                 'got {0}'.format(type(ens).__name__))
        container['ensembles'].append(ens.get_dict_representation())
    for seq in (sequences or ()):
        if not isinstance(seq, PulseSequence):
            raise PulseJsonError('sequences entries must be PulseSequence instances, got {0}'
                                 ''.format(type(seq).__name__))
        container['sequences'].append(seq.get_dict_representation())

    json_str = json.dumps(container, indent=2, allow_nan=False, default=_json_default)
    if file_path is not None:
        with open(file_path, 'w', encoding='utf-8') as fh:
            fh.write(json_str)
    return json_str


# ---------------------------------------------------------------------------- import helpers

def _known_sampling_functions():
    if not SamplingFunctions.parameters:
        # The qudi manager console runs in a SEPARATE process (namespace server / rpyc), so
        # the import_sampling_functions() call done by SequenceGeneratorLogic.on_activate in
        # the qudi process is invisible here — class-level state does not cross processes.
        # Lazily load the DEFAULT namespace set so validation works out-of-process too.
        # NOTE: sampling functions added via the logic's additional-paths ConfigOption are
        # NOT loaded by this fallback; files using such extensions must be imported in a
        # process where those paths were imported (the rejection error stays loud + exact).
        SamplingFunctions.import_sampling_functions([])
    return set(SamplingFunctions.parameters)


def _validate_element_dict(element_dict, where, allow_label=False):
    if not isinstance(element_dict, dict):
        raise PulseJsonError('{0}: element entries must be JSON objects.'.format(where))
    keys = set(element_dict)
    if allow_label:
        _check_label(element_dict, where)   # v2: optional display label, must be str
        keys = keys - {'label'}
    if keys != _ELEMENT_KEYS:
        extra = ' A v2 "label" is the ONLY extra key allowed; ' if not allow_label else ' '
        raise PulseJsonError(
            '{0}: element DATA keys must be exactly {1}; got {2}.{3}(Contract §3 — the '
            'element DATA dict feeds the PulseBlockElement constructor verbatim.)'.format(
                where, sorted(_ELEMENT_KEYS), sorted(keys), extra))
    known = _known_sampling_functions()
    pf = element_dict['pulse_function']
    if not isinstance(pf, dict):
        raise PulseJsonError('{0}: pulse_function must be a JSON object.'.format(where))
    for chnl, sf in pf.items():
        if (not isinstance(sf, dict)) or set(sf) != {'name', 'params'}:
            raise PulseJsonError(
                '{0}: pulse_function["{1}"] must be an object with exactly the keys '
                '"name" and "params".'.format(where, chnl))
        if sf['name'] not in known:
            raise PulseJsonError(
                '{0}: unknown sampling function "{1}" on channel {2}. Known functions on '
                'this installation: {3}. REJECTING the file (contract §3.1: never guess).'
                ''.format(where, sf['name'], chnl, sorted(known)))
        if not isinstance(sf['params'], dict):
            raise PulseJsonError('{0}: sampling-function params must be a JSON object.'
                                 ''.format(where))


def _normalize_block_list(block_list, where):
    if not isinstance(block_list, list):
        raise PulseJsonError('{0}: block_list must be an array.'.format(where))
    normalized = []
    for entry in block_list:
        if (not isinstance(entry, (list, tuple))) or len(entry) != 2:
            raise PulseJsonError('{0}: block_list entries must be [name, repetitions] '
                                 'pairs.'.format(where))
        name, reps = entry
        if not isinstance(name, str):
            raise PulseJsonError('{0}: block_list names must be strings.'.format(where))
        if isinstance(reps, bool) or not isinstance(reps, numbers.Integral) or reps < 0:
            raise PulseJsonError('{0}: block_list repetitions must be integers >= 0 '
                                 '(got {1!r} for block "{2}").'.format(where, reps, name))
        normalized.append((name, int(reps)))
    return normalized


def _read_raw_text(source):
    """Return the raw JSON text for a source that is either a JSON string (leading '{' after
    strip) or a file path. Shared by import_from_json and play_ready's collision pre-scan."""
    text = source.strip() if isinstance(source, str) else ''
    if text.startswith('{'):
        return text
    with open(source, 'r', encoding='utf-8') as fh:
        return fh.read()


def import_from_json(source, sequence_generator_logic=None, save=True):
    """Import a format_version 1 OR 2 JSON container into pulse objects.

    v2 optional display labels on elements/blocks are validated (must be str) and STRIPPED
    before construction — qudi objects never carry them. v1 files stay strict (no labels).

    @param str source: a file path OR a JSON string (detected by leading '{' after strip)
    @param SequenceGeneratorLogic sequence_generator_logic: target logic; required when
           save=True and used to resolve references against already-saved assets
    @param bool save: hand every created object to the stock save path
           (save_block / save_ensemble / save_sequence)
    @return dict: {'blocks': [...], 'ensembles': [...], 'sequences': [...]} created objects
    """
    raw = _read_raw_text(source)
    try:
        container = json.loads(raw)
    except json.JSONDecodeError as err:
        raise PulseJsonError('Not valid JSON: {0}'.format(err)) from err

    if not isinstance(container, dict):
        raise PulseJsonError('Top level must be a JSON object (contract §1).')
    unknown_keys = set(container) - _CONTAINER_KEYS
    if unknown_keys:
        raise PulseJsonError('Unknown top-level keys {0} (contract §1/§8: tool metadata '
                             'belongs in generation_method_parameters).'
                             ''.format(sorted(unknown_keys)))
    version = container.get('format_version')
    if not isinstance(version, int):
        raise PulseJsonError('format_version (int) is required (contract §1).')
    if version > FORMAT_VERSION:
        raise PulseJsonError('format_version {0} is newer than this importer (v{1}) — '
                             'REJECTING (contract §1).'.format(version, FORMAT_VERSION))
    allow_label = version >= 2   # v1 stays STRICT: no labels; v2 permits optional labels

    if save and sequence_generator_logic is None:
        raise PulseJsonError('save=True requires a SequenceGeneratorLogic instance.')

    created = {'blocks': [], 'ensembles': [], 'sequences': []}

    # ---- blocks (dependency order, contract §7)
    block_names = set()
    for blk_dict in container.get('blocks', []):
        block_keys = {'name', 'element_list'} | ({'label'} if allow_label
                                                 and 'label' in blk_dict else set())
        if not isinstance(blk_dict, dict) or set(blk_dict) != block_keys:
            raise PulseJsonError('Block entries must carry exactly "name" and '
                                 '"element_list"'
                                 + (' (plus an optional v2 "label")' if allow_label else '')
                                 + ' (contract §4).')
        where = 'block "{0}"'.format(blk_dict.get('name'))
        if allow_label:
            _check_label(blk_dict, where)
        if not isinstance(blk_dict['element_list'], list):
            raise PulseJsonError('{0}: element_list must be an array.'.format(where))
        for ii, element_dict in enumerate(blk_dict['element_list']):
            _validate_element_dict(element_dict, '{0} element {1}'.format(where, ii),
                                   allow_label=allow_label)
        # deepcopy + strip labels: the stock *_from_dict mutate their input in place and
        # PulseBlock/PulseBlockElement(**d) would choke on a 'label' kwarg.
        block = PulseBlock.block_from_dict(_strip_labels(copy.deepcopy(blk_dict)))
        block_names.add(block.name)
        created['blocks'].append(block)

    # ---- ensembles
    ensemble_names = set()
    for ens_dict in container.get('ensembles', []):
        required = {'name', 'rotating_frame', 'block_list', 'sampling_information',
                    'measurement_information', 'generation_method_parameters'}
        if not isinstance(ens_dict, dict) or set(ens_dict) != required:
            raise PulseJsonError('Ensemble entries must carry exactly {0} (contract §5).'
                                 ''.format(sorted(required)))
        where = 'ensemble "{0}"'.format(ens_dict.get('name'))
        work = copy.deepcopy(ens_dict)
        work['block_list'] = _normalize_block_list(work['block_list'], where)
        for name, _ in work['block_list']:
            if name in block_names:
                continue
            if sequence_generator_logic is not None and \
                    name in sequence_generator_logic.saved_pulse_blocks:
                continue
            raise PulseJsonError('{0} references block "{1}" which is neither in this file '
                                 'nor already saved — REJECTING (contract §7).'
                                 ''.format(where, name))
        ensemble = PulseBlockEnsemble.ensemble_from_dict(work)
        ensemble_names.add(ensemble.name)
        created['ensembles'].append(ensemble)

    # ---- sequences
    for seq_dict in container.get('sequences', []):
        required = {'name', 'rotating_frame', 'ensemble_list', 'sampling_information',
                    'measurement_information', 'generation_method_parameters'}
        if not isinstance(seq_dict, dict) or set(seq_dict) != required:
            raise PulseJsonError('Sequence entries must carry exactly {0} (contract §6).'
                                 ''.format(sorted(required)))
        where = 'sequence "{0}"'.format(seq_dict.get('name'))
        if not isinstance(seq_dict['ensemble_list'], list):
            raise PulseJsonError('{0}: ensemble_list must be an array.'.format(where))
        for ii, step in enumerate(seq_dict['ensemble_list']):
            if not isinstance(step, dict) or not isinstance(step.get('ensemble'), str):
                raise PulseJsonError('{0} step {1}: each step must be an object with a '
                                     'string "ensemble" key (contract §6).'.format(where, ii))
            ref = step['ensemble']
            if ref in ensemble_names:
                continue
            if sequence_generator_logic is not None and \
                    ref in sequence_generator_logic.saved_pulse_block_ensembles:
                continue
            raise PulseJsonError('{0} step {1} references ensemble "{2}" which is neither '
                                 'in this file nor already saved — REJECTING (contract §7).'
                                 ''.format(where, ii, ref))
        # SequenceStep construction (via PulseSequence.insert) validates the step keys
        sequence = PulseSequence.sequence_from_dict(copy.deepcopy(seq_dict))
        created['sequences'].append(sequence)

    if save:
        for block in created['blocks']:
            sequence_generator_logic.save_block(block)
        for ensemble in created['ensembles']:
            sequence_generator_logic.save_ensemble(ensemble)
        for sequence in created['sequences']:
            sequence_generator_logic.save_sequence(sequence)

    return created


# ------------------------------------------------------------ play-ready (T24, one call)

# HARD BOUNDARY (consultant T24, owner-approved 2026-08-15): this string is embedded in the
# docstring AND in every summary this function returns. Changing it is a policy change.
_NO_OUTPUT_NOTE = (
    'STOPPED at "loaded, ready to play". Output-enable is NOT part of this call: turning the '
    'pulser output ON is a deliberate human/GUI action under the setup safety chain '
    '(SAFE-005 / RF-ON discipline). No argument to play_ready() enables the output, and it '
    'never calls pulser_on / set_status / any output-enable path.')


# TRANSIENT-import re-sample limitation (consultant T26, owner-approved 2026-08-16): embedded
# in the docstring AND returned in summary['transient_note'] whenever transient removal ran.
_TRANSIENT_NOTE = (
    'TRANSIENT: the block(s) this call added to the saved pool were REMOVED after load (memory '
    "+ disk) so the saved-blocks list is not polluted. LIMITATION: a transient asset cannot be "
    'RE-SAMPLED / regenerated without RE-IMPORTING the JSON file — the loaded waveform can still '
    'be REPLAYED as-is (it lives in pulser memory, untouched), but qudi has no construction plan '
    'to rebuild it from. Keep the .pulse.json if you may need to re-sample.')


def play_ready(file_path, sequence_generator_logic, assets=None, transient=False):
    """Import a pulse-JSON file and take its assets all the way to LOADED-AND-READY — sampled
    onto the pulse generator and loaded into its channels — WITHOUT ever enabling the output.

    Pipeline per asset: import_from_json (validate + save) -> sample_pulse_block_ensemble /
    sample_pulse_sequence -> load_ensemble / load_sequence. It stops there. The pulser is left
    with a waveform/sequence loaded but its output OFF.

    HARD BOUNDARY (T24): output-enable is NOT part of this call. Turning the pulser output ON
    is a deliberate human/GUI action under the setup safety chain (SAFE-005 / RF-ON
    discipline). No argument to play_ready() enables the output; it never calls pulser_on /
    set_status / any output-enable path. The same statement is returned in summary['note'] and
    summary['output_enabled'] is always False.

    TRANSIENT import (T26): with transient=True, after everything is loaded the block(s) THIS
    call added to the saved pool are removed (memory + disk) so the saved-blocks list is not
    polluted; ensembles/sequences are KEPT (their measurement_information + invoke/generation
    settings stay usable). transient='all' also removes the ensembles/sequences this call
    imported. transient=False (default) leaves everything saved — behaviour identical to T24.

      * Only assets THIS call introduced are ever removed. The removal set is computed as a
        before/after snapshot difference of the pool, so (a) nothing pre-existing is touched,
        and (b) any helper block qudi's sampler injects to meet waveform granularity
        ('idle_extension') is also cleaned up. To keep that guarantee airtight, a transient
        import first REJECTS (before importing anything) if any name in the file collides with
        an already-saved asset — so import can never overwrite, and later remove, a pre-existing
        asset.
      * LIMITATION (documented, prominent): a transient asset CANNOT be re-sampled/regenerated
        without re-importing the JSON. The loaded waveform replays fine (pulser memory is not
        touched); only qudi's construction plan is gone. See summary['transient_note'].

    Failure containment: import is all-or-nothing (a single unknown sampling function rejects
    the whole file before anything is saved or sampled). After import, each asset is sampled
    then loaded independently inside its own try/except; a failure on one asset never leaves
    another in a half-known state. The returned summary reports, per asset, exactly whether it
    is saved / sampled / loaded, so nothing is ambiguous.

    @param str file_path: path to a .pulse.json file (or a JSON string — same detection as
           import_from_json)
    @param SequenceGeneratorLogic sequence_generator_logic: a RUNNING SequenceGeneratorLogic
           (its pulsegenerator() is the device that gets the waveforms/sequences). Required.
    @param list assets: optional list of ensemble/sequence NAMES to sample+load. Default None
           = every ensemble and every sequence imported from the file. Blocks are never
           sampled/loaded directly (they are building blocks); a name that is neither a saved
           ensemble nor a saved sequence after import is reported as an error, not guessed.
    @param transient: False (default) keep everything saved; True remove this call's blocks
           after load (keep ensembles/sequences); 'all' also remove this call's
           ensembles/sequences. Any other value is rejected.
    @return dict: summary with keys
            'file', 'output_enabled' (always False), 'imported' {blocks,ensembles,sequences},
            'requested' (asset names acted on), 'assets' (per-asset records with
            type/saved/sampled/loaded/sample_s/load_s/error), 'ready_to_play' (names loaded),
            'ok' (bool: every requested asset loaded), 'note' (the hard-boundary text),
            'transient' (the mode), 'removed' {blocks,ensembles,sequences}, and
            'transient_note' (the re-sample limitation text; '' when transient=False).
    """
    if sequence_generator_logic is None:
        raise PulseJsonError('play_ready requires a running SequenceGeneratorLogic instance '
                             '(sampling and loading target its pulsegenerator()).')
    if transient not in (False, True, 'all'):
        raise PulseJsonError("transient must be False, True, or 'all'; got {0!r}."
                             ''.format(transient))
    sgl = sequence_generator_logic

    # ---- transient guard: reject BEFORE importing if any name in the file collides with an
    # already-saved asset, so import can never overwrite (and transient-removal never delete) a
    # pre-existing asset. Non-transient imports keep the stock overwrite behaviour (T24).
    if transient:
        try:
            container = json.loads(_read_raw_text(file_path))
        except (json.JSONDecodeError, OSError) as err:
            raise PulseJsonError('transient import could not read/parse the file for the '
                                 'collision pre-check: {0}'.format(err)) from err
        if isinstance(container, dict):
            collisions = []
            for key, pool in (('blocks', sgl.saved_pulse_blocks),
                              ('ensembles', sgl.saved_pulse_block_ensembles),
                              ('sequences', sgl.saved_pulse_sequences)):
                singular = key[:-1]   # blocks->block, ensembles->ensemble, sequences->sequence
                for entry in container.get(key, []) or []:
                    nm = entry.get('name') if isinstance(entry, dict) else None
                    if nm in pool:
                        collisions.append('{0} "{1}"'.format(singular, nm))
            if collisions:
                raise PulseJsonError(
                    'transient import REFUSED — the file collides with already-saved asset(s): '
                    '{0}. A transient import must not overwrite/remove anything pre-existing; '
                    'rename in the file or delete the saved asset(s) first.'
                    ''.format(', '.join(collisions)))

    # snapshot the block pool so we can remove EXACTLY what this call adds (imported blocks +
    # any sampler-injected idle_extension), never a pre-existing block.
    blocks_before = set(sgl.saved_pulse_blocks)
    ensembles_before = set(sgl.saved_pulse_block_ensembles)
    sequences_before = set(sgl.saved_pulse_sequences)

    # ---- stage 1: import (all-or-nothing; save=True so the stock save path persists them).
    # A rejected file raises here — nothing sampled, nothing loaded (acceptance case b).
    created = import_from_json(file_path, sgl, save=True)
    imported = {'blocks': [b.name for b in created['blocks']],
                'ensembles': [e.name for e in created['ensembles']],
                'sequences': [s.name for s in created['sequences']]}

    # ---- resolve which assets to sample+load
    if assets is None:
        requested = [(n, 'ensemble') for n in imported['ensembles']] + \
                    [(n, 'sequence') for n in imported['sequences']]
    else:
        requested = []
        for name in assets:
            if name in sgl.saved_pulse_block_ensembles:
                requested.append((name, 'ensemble'))
            elif name in sgl.saved_pulse_sequences:
                requested.append((name, 'sequence'))
            else:
                requested.append((name, 'unknown'))

    summary = {'file': file_path if isinstance(file_path, str)
               and not file_path.strip().startswith('{') else '<json-string>',
               'output_enabled': False,   # HARD BOUNDARY — never flips true in this function
               'imported': imported,
               'requested': [n for n, _ in requested],
               'assets': [],
               'ready_to_play': [],
               'ok': True,
               'note': _NO_OUTPUT_NOTE,
               'transient': transient,
               'removed': {'blocks': [], 'ensembles': [], 'sequences': []},
               'transient_note': ''}

    # ---- stage 2+3: sample then load, per asset, each isolated
    for name, kind in requested:
        rec = {'name': name, 'type': kind, 'saved': kind in ('ensemble', 'sequence'),
               'sampled': False, 'loaded': False, 'sample_s': None, 'load_s': None,
               'error': None}
        try:
            if kind == 'unknown':
                raise PulseJsonError(
                    'asset "{0}" is neither a saved ensemble nor a saved sequence after '
                    'import — not sampling/loading a guessed asset.'.format(name))
            elif kind == 'ensemble':
                t0 = time.perf_counter()
                offset_bin, waveforms, _info = sgl.sample_pulse_block_ensemble(name)
                rec['sample_s'] = time.perf_counter() - t0
                # sample_pulse_block_ensemble returns (-1, [], {}) on failure
                rec['sampled'] = (offset_bin != -1) and bool(waveforms)
                if not rec['sampled']:
                    raise PulseJsonError('sampling of ensemble "{0}" failed (see the qudi log '
                                         'for the reason); NOT loading.'.format(name))
                t0 = time.perf_counter()
                ret = sgl.load_ensemble(name)
                rec['load_s'] = time.perf_counter() - t0
                if ret == -1:
                    raise PulseJsonError('load refused for ensemble "{0}": the pulser is '
                                         'already running — switch the output OFF and retry '
                                         '(play_ready never turns it on).'.format(name))
                rec['loaded'] = (sgl.loaded_asset == (name, 'PulseBlockEnsemble'))
                if not rec['loaded']:
                    raise PulseJsonError('ensemble "{0}" did not become the loaded asset '
                                         '(loaded_asset={1}).'.format(name, sgl.loaded_asset))
            else:  # sequence
                t0 = time.perf_counter()
                sgl.sample_pulse_sequence(name)   # returns None on success; check post-cond
                rec['sample_s'] = time.perf_counter() - t0
                rec['sampled'] = name in sgl.sampled_sequences
                if not rec['sampled']:
                    raise PulseJsonError('sampling of sequence "{0}" failed (see the qudi log '
                                         'for the reason); NOT loading.'.format(name))
                t0 = time.perf_counter()
                ret = sgl.load_sequence(name)
                rec['load_s'] = time.perf_counter() - t0
                if ret == -1:
                    raise PulseJsonError('load refused for sequence "{0}": the pulser is '
                                         'already running — switch the output OFF and retry '
                                         '(play_ready never turns it on).'.format(name))
                rec['loaded'] = (sgl.loaded_asset == (name, 'PulseSequence'))
                if not rec['loaded']:
                    raise PulseJsonError('sequence "{0}" did not become the loaded asset '
                                         '(loaded_asset={1}).'.format(name, sgl.loaded_asset))
        except Exception as err:
            rec['error'] = str(err)
        summary['assets'].append(rec)
        if rec['loaded']:
            summary['ready_to_play'].append(name)
        else:
            summary['ok'] = False

    # ---- stage 4 (transient, T26): drop this call's construction plan from the saved pool.
    # The loaded waveform stays in pulser memory (replayable); only the qudi-side objects go.
    if transient:
        # blocks: remove EXACTLY what this call added to the pool = imported blocks + any
        # sampler-injected idle_extension. Snapshot difference => never a pre-existing block.
        blocks_to_remove = set(sgl.saved_pulse_blocks) - blocks_before
        for name in sorted(blocks_to_remove):
            sgl.delete_block(name)
            summary['removed']['blocks'].append(name)
        if transient == 'all':
            for name in sorted(set(sgl.saved_pulse_block_ensembles) - ensembles_before):
                sgl.delete_ensemble(name)
                summary['removed']['ensembles'].append(name)
            for name in sorted(set(sgl.saved_pulse_sequences) - sequences_before):
                sgl.delete_sequence(name)
                summary['removed']['sequences'].append(name)
        summary['transient_note'] = _TRANSIENT_NOTE

    return summary


def _format_play_ready_summary(summary):
    """Render a play_ready() summary as human-readable lines (used by the --load CLI)."""
    lines = ['play_ready: {0}'.format(summary['file']),
             '  imported: {0} block(s), {1} ensemble(s), {2} sequence(s)'.format(
                 len(summary['imported']['blocks']), len(summary['imported']['ensembles']),
                 len(summary['imported']['sequences'])),
             '  output_enabled: {0}  (HARD BOUNDARY — always False)'.format(
                 summary['output_enabled'])]
    for rec in summary['assets']:
        def _ms(v):
            return '{0:.0f} ms'.format(v * 1e3) if isinstance(v, float) else '-'
        status = 'READY' if rec['loaded'] else 'NOT READY'
        lines.append('  [{0:9s}] {1} "{2}"  saved={3} sampled={4}({5}) loaded={6}({7})'.format(
            status, rec['type'], rec['name'], rec['saved'], rec['sampled'], _ms(rec['sample_s']),
            rec['loaded'], _ms(rec['load_s'])))
        if rec['error']:
            lines.append('              error: {0}'.format(rec['error']))
    lines.append('  ready_to_play: {0}'.format(summary['ready_to_play'] or '(none)'))
    if summary.get('transient'):
        rem = summary['removed']
        lines.append('  transient={0} — removed from saved pool: {1} block(s){2}'.format(
            summary['transient'], len(rem['blocks']),
            '' if summary['transient'] != 'all' else
            ', {0} ensemble(s), {1} sequence(s)'.format(len(rem['ensembles']),
                                                        len(rem['sequences']))))
        if summary.get('transient_note'):
            lines.append('  {0}'.format(summary['transient_note']))
    lines.append('  {0}'.format(summary['note']))
    return '\n'.join(lines)


# ---------------------------------------------------------------------------- CLI (T20)

def _cli_persist(created, assets_dir=None):
    """Persist imported objects into the qudi saved-assets dir via the STOCK
    SequenceGeneratorLogic pickle helpers, WITHOUT a running qudi (no Module activation).

    It borrows the unbound stock ``_save_*_to_file`` methods — i.e. qudi's own pickle
    code writes the .block/.ensemble/.sequence files, the pickle boundary is untouched —
    and provides only the two attributes those methods use (``_assets_storage_dir`` and
    ``log``). Returns the assets directory used. Default dir mirrors the logic's
    ConfigOption default (<home>/saved_pulsed_assets); override with --assets-dir when the
    setup's config sets a custom assets_storage_path.
    """
    import logging
    from qudi.logic.pulsed.sequence_generator_logic import SequenceGeneratorLogic
    from qudi.util.paths import get_home_dir
    directory = assets_dir or os.path.join(get_home_dir(), 'saved_pulsed_assets')
    os.makedirs(directory, exist_ok=True)

    class _Saver:
        pass
    saver = _Saver()
    saver._assets_storage_dir = directory
    saver.log = logging.getLogger('qudi.pulsed_json_io.cli')
    for block in created['blocks']:
        SequenceGeneratorLogic._save_block_to_file(saver, block)
    for ensemble in created['ensembles']:
        SequenceGeneratorLogic._save_ensemble_to_file(saver, ensemble)
    for sequence in created['sequences']:
        SequenceGeneratorLogic._save_sequence_to_file(saver, sequence)
    return directory


# The supported route for --load: run inside the RUNNING qudi manager's console, where the
# live SequenceGeneratorLogic instance is already in scope. This is deliberately NOT an
# out-of-process rpyc attach — qudi shares logic modules over rpyc only when explicitly
# configured, its device calls are netobtain()-proxied, and half-driving a real pulser from a
# separate process is exactly what the setup safety chain (SAFE-005 / RF-ON) guards against.
_LOAD_CONSOLE_ROUTE = (
    "--load cannot run headless: sampling+loading needs the RUNNING qudi's live pulse\n"
    "generator. The supported route is the qudi manager console (where the logic module is\n"
    "already in scope). Run there:\n\n"
    "    from qudi.logic.pulsed.pulsed_json_io import play_ready\n"
    "    summary = play_ready(r'{path}', sequencegeneratorlogic)\n"
    "    print(summary['ok'], summary['ready_to_play'])\n\n"
    "It imports, samples, and loads — and STOPS there. It never enables the output; turning\n"
    "the pulser ON stays a deliberate GUI/human action (SAFE-005 / RF-ON discipline).\n"
    "To persist assets WITHOUT a running qudi, drop --load: import-only pickles them into the\n"
    "saved-assets dir and the GUI picks them up at next start / via 'Import JSON…' refresh.")


def _main(argv):
    """CLI: python -m qudi.logic.pulsed.pulsed_json_io <file.pulse.json> [more...] [--load]

    Default (no --load): imports each file through the SAME validated import_from_json path
    (all-or-nothing per file: a rejected file writes nothing) and persists via the stock
    pickle helpers into the saved-assets dir — no running qudi required. GUI lists refresh at
    the next qudi start or via the pulsed GUI 'Import JSON…' refresh. Per-file OK/FAIL
    summary; nonzero exit on any failure.

    --load: take assets all the way to loaded-and-ready (import -> sample -> load) via
    play_ready(). This needs a RUNNING qudi and its live pulse generator, which is only
    reachable from the qudi manager console — so the CLI prints the exact console one-liner
    and exits nonzero rather than attempting a fragile/unsafe out-of-process attach. --load
    NEVER enables the pulser output. Import-only still works by omitting --load.
    """
    import argparse
    parser = argparse.ArgumentParser(
        prog='python -m qudi.logic.pulsed.pulsed_json_io',
        description='Import pulse-sequence JSON file(s) into the qudi saved-assets '
                    'directory (no running qudi needed). Validates via the same path the '
                    'GUI uses; v2 display labels are stripped (qudi objects carry none). '
                    'Use --load to go all the way to loaded-and-ready via the qudi console.')
    parser.add_argument('files', nargs='+', help='.pulse.json file(s) to import')
    parser.add_argument('--assets-dir', default=None,
                        help='override the saved_pulsed_assets directory')
    parser.add_argument('--load', action='store_true',
                        help='sample + load onto the pulser (loaded, NOT playing) — requires '
                             'a running qudi; prints the supported console one-liner. Never '
                             'enables the output.')
    args = parser.parse_args(argv)

    if args.load:
        # Loading needs the running qudi's live device; the CLI cannot obtain it headless.
        # Emit the supported console route per file and fail clearly (import-only still works).
        for path in args.files:
            print('LOAD  {0}'.format(path))
            print(_LOAD_CONSOLE_ROUTE.format(path=path))
            print('')
        print('--load is not available headless — see the console route above. '
              '(import-only works: re-run without --load.)')
        return 2

    n_ok = n_fail = 0
    for path in args.files:
        try:
            created = import_from_json(path, None, save=False)   # validate + build only
            directory = _cli_persist(created, args.assets_dir)   # stock pickle helpers
            print('OK    {0}  ->  {1} block(s), {2} ensemble(s), {3} sequence(s)  [{4}]'
                  ''.format(path, len(created['blocks']), len(created['ensembles']),
                            len(created['sequences']), directory))
            n_ok += 1
        except Exception as err:   # PulseJsonError or IO/OS error — report, do not persist
            print('FAIL  {0}  ->  {1}'.format(path, err))
            n_fail += 1
    print('\n{0} ok, {1} failed'.format(n_ok, n_fail))
    return 1 if n_fail else 0


if __name__ == '__main__':
    import sys
    sys.exit(_main(sys.argv[1:]))
