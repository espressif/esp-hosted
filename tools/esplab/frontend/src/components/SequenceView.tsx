import { useEffect, useRef } from 'react'
import mermaid from 'mermaid'

mermaid.initialize({
  startOnLoad: false,
  theme: 'dark',
  securityLevel: 'loose',
  sequence: { useMaxWidth: true, mirrorActors: false, messageFontSize: 12, actorFontSize: 13 },
})

let idc = 0

// Central, event-driven view of the running test: a sequence diagram generated
// from the live message stream (Mermaid). Re-renders when the source changes.
export default function SequenceView({ text }: { text: string }) {
  const ref = useRef<HTMLDivElement>(null)
  useEffect(() => {
    let dead = false
    mermaid.render('seq' + idc++, text)
      .then(({ svg }) => { if (!dead && ref.current) ref.current.innerHTML = svg })
      .catch(() => { /* transient parse during edits; ignore */ })
    return () => { dead = true }
  }, [text])
  return <div ref={ref} className="w-full h-full overflow-auto p-4 grid place-items-start [&_svg]:mx-auto" />
}
