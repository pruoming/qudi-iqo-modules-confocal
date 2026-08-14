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

__all__ = ['FORMAT_VERSION', 'PulseJsonError', 'export_to_json', 'import_from_json']

import copy
import json
import numbers

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
    text = source.strip() if isinstance(source, str) else ''
    if text.startswith('{'):
        raw = text
    else:
        with open(source, 'r', encoding='utf-8') as fh:
            raw = fh.read()
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
