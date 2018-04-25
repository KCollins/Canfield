#!/usr/bin/env python2

import ast
import math
from sys import stdin, stdout, stderr

def demodulate(module):
	return { k: getattr(module, k) for k in dir(module) }

def parse_param(kv):
	k, s, v = kv.partition('=')

	if not s:
		m = "Expected something like \"k=v\", but missing '='."
		raise ValueError(m)

	if not v:
		raise ValueError("No value specified for %s parameter" % (k,))

	# Is `k' a valid identifier?
	ast.parse('%s = %r' % (k, None))

	return k, ast.literal_eval(v)

def separate(infile):
	b = ''     # Output buffer
	e = False  # Escaping next character, used when m is 'text'
	i = 0      # How far back to the last escaped character?
	n = []     # Current nestings, used when m is 'expr' or 'code'
	m = 'text' # Current mode
	s = ()     # Whitespace sequences to strip if found

	sentinels = { '`':  'expr'
	            , '$(': 'expr'
	            , '${': 'code'
	            }

	groups = { '(': ')'
	         , '[': ']'
	         , '{': '}'
	         }
	inv_groups = { v: k for k, v in groups.iteritems() }

	while True:
		c = infile.read(1)
		if not c: break

		if m == 'text':
			if s:
				if c in s:
					s = filter(bool, (w[1:] for w in s if w.startswith(c)))
					continue
				else:
					s = ()

			if e:
				b += c
				e  = False
				i  = 0
				continue
			elif c == '\\':
				e = True
				continue

			b += c
			i += 1

			for s_start, s_mode in sentinels.iteritems():
				if b[-i:].endswith(s_start): break
			else:
				continue

			n.append(c)
			yield m, b[:-len(s_start)]
			m, b = s_mode, ''
		elif m in ('expr', 'code'):
			if c == '#':
				while True:
					d = infile.read(1)
					if not d: break
					c += d
					if d in ('\r', '\n', '\f'):
						break
			elif c in ('"', '\''):
				while True:
					d = infile.read(1)
					if not d: break
					c += d
					if d != c[0]: continue
					try:
						v = ast.literal_eval(c)
					except (SyntaxError, TypeError, ValueError):
						continue
					if isinstance(v, basestring):
						break
			elif c == '`' and n == ['`']:
				n = []
				c = ''
			elif c in groups:
				n.append(c)
			elif c in inv_groups:
				if not n or n.pop() != inv_groups[c]:
					raise SyntaxError("Premature %r" % (c,))

			if n:
				b += c
			else:
				yield m, b
				if m == 'code':
					s = '\r', '\n', '\r\n'
				m, b = 'text', ''

	if m != 'text':
		raise SyntaxError("Unterminated %r block." % (m,))
	if e:
		raise SyntaxError("Input terminated by incomplete escape sequence.")

	yield m, b

# TODO require caller to redirect infile to stdin and outfile to stdout,
# because evaluated Python 2 code can't be easily tricked into `print'-ing to
# outfile instead of stdout.
#
# TODO alternately, convert all of this to Python 3.
def xfm_template(infile, outfile, params=()):
	gls = {}
	gls.update(demodulate(math))
	gls.update(tau=(math.pi * 2))

	lcs = {k: v for k, v in map(parse_param, params)}

	def run(code, mode):
		r = eval(compile(code, infile.name, mode), gls, lcs)
		#print "\033[31mresult: %r\033[0m" % (r,)
		return r

	for seg_type, seg_text in separate(infile):
		#print "\033[32m%s: %r\033[0m" % (seg_type, seg_text)
		if seg_type == 'text':
			outfile.write(seg_text)
		elif seg_type == 'expr':
			outfile.write(str(run(seg_text, 'eval')))
		elif seg_type == 'code':
			run(seg_text, 'exec')

if __name__ == '__main__':
	import argparse

	p = argparse.ArgumentParser(description="Canfield Joint SDF Generator")

	p.add_argument( '-i', '--infile'
	              , default='-'
	              , help="Where to read the SDF template (- for standard in)"
	              )
	p.add_argument( '-o', '--outfile'
	              , default='-'
	              , help="Where to write the output SDF (- for standard out)"
	              )

	p.add_argument( 'params'
	              , metavar='VARIABLE=VALUE'
	              , nargs='*'
	              , help="A variable to substitute into the template"
	              )

	args = p.parse_args()

	class MaybeOpen(object):
		def __init__(self, f, mode='r', default=None, **kwargs):
			super(MaybeOpen, self).__init__(**kwargs)
			self.f     = f if f != '-' else default
			self.openp = self.f not in (stdin, stdout, stderr)
			self.mode  = mode

		def __enter__(self):
			if self.openp:
				self.f = open(self.f, self.mode)
				return self.f.__enter__()
			else:
				return self.f

		def __exit__(self, *args):
			if self.openp:
				self.f.__exit__(*args)

	with MaybeOpen(args.infile,  mode='r', default=stdin)  as infile \
	   , MaybeOpen(args.outfile, mode='w', default=stdout) as outfile:
		xfm_template(infile, outfile, args.params)
