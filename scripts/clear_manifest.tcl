# clear_manifest.tcl — put the board back to a known state.
#
#   xsdb scripts/clear_manifest.tcl
#
# DDR does not reliably clear on a power cycle: DRAM cells hold charge long
# enough to survive one, so a manifest can outlive the frames it describes and
# a later run would encode stale data while reporting success.
#
# There is no need to wipe DDR to fix that. The application gates everything on
# the manifest's magic number, so zeroing that one word makes it wait for a
# fresh load, and whatever remains underneath is overwritten by the next load
# anyway. This clears the whole manifest header rather than only the magic, so
# a half-written manifest cannot look plausible either.
#
# The application also clears the magic itself once it has latched a manifest,
# so this is only needed after an interrupted load or a crashed run.

set MANIFEST 0x03000000

connect
targets -set -filter {name =~ "ARM*#0"}
stop

# 1 KB of zeros: the 32-byte header plus room for the frame records.
mwr -force $MANIFEST 0 256
puts "cleared manifest header at [format 0x%08X $MANIFEST]"

# Read it back. mwr syncs caches by default, but confirming beats assuming.
set got [lindex [mrd -value $MANIFEST 1] 0]
if {$got == 0} {
    puts "verified: magic reads 0, the application will wait for a fresh load"
} else {
    puts "WARNING: magic still reads [format 0x%08X $got] -- the write did not take"
}

con
puts "core resumed"
