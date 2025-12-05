import { CopyButton } from "@/components/common/copy-button";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Loader2, TerminalSquare, XCircle } from "lucide-react";
import { useEffect, useRef, useState } from "react";

interface LogStreamProps {
  logFile: string | null;
  isLoading: boolean;
  onClose?: () => void;
}

export const LogStream = ({ logFile, isLoading, onClose }: LogStreamProps) => {
  const [logs, setLogs] = useState<string>("");
  const [connected, setConnected] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const logContainerRef = useRef<HTMLDivElement>(null);
  const abortControllerRef = useRef<AbortController | null>(null);

  useEffect(() => {
    if (!logFile) return;

    const fetchLogs = async () => {
      try {
        // Create a new AbortController for this request
        abortControllerRef.current = new AbortController();
        const signal = abortControllerRef.current.signal;

        setConnected(true);
        setError(null);

        const response = await fetch(`/training/logs/${logFile}`, {
          signal,
          headers: {
            Accept: "text/plain",
          },
        });

        if (!response.ok) {
          throw new Error(`Failed to fetch logs: ${response.statusText}`);
        }

        // Get the reader from the response body stream
        const reader = response.body?.getReader();
        if (!reader) {
          throw new Error("Stream reader not available");
        }

        // Read the stream
        const decoder = new TextDecoder();
        let accumulatedLogs = "";

        while (true) {
          const { done, value } = await reader.read();
          if (done) break;

          // Decode and append the new chunk
          const chunk = decoder.decode(value, { stream: true });
          accumulatedLogs += chunk;
          setLogs(accumulatedLogs);

          // Scroll to bottom
          if (logContainerRef.current) {
            logContainerRef.current.scrollTop =
              logContainerRef.current.scrollHeight;
          }
        }
      } catch (err) {
        setError(err instanceof Error ? err.message : "Error fetching logs");
        console.error("Error streaming logs:", err);
      } finally {
        setConnected(false);
      }
    };

    fetchLogs();

    // Cleanup function
    return () => {
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
      }
    };
  }, [logFile]);

  // Auto-scroll to bottom when logs update
  useEffect(() => {
    if (logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  }, [logs]);

  const handleClose = () => {
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
    }
    if (onClose) onClose();
  };

  if (!logFile && !isLoading) return null;

  return (
    <Card className="mt-4 p-4">
      <div className="flex justify-between items-center mb-2">
        <div className="flex items-center">
          <TerminalSquare className="size-4 mr-2" />
          <span className="font-medium">Training Logs</span>
          {connected && (
            <Loader2 className="size-4 ml-2 animate-spin text-blue-500" />
          )}
        </div>
        <div className="flex gap-2">
          {logs && <CopyButton text={logs} hint="Copy logs" variant="ghost" />}
          <Button variant="ghost" size="sm" onClick={handleClose}>
            <XCircle className="size-4" />
          </Button>
        </div>
      </div>

      <div
        ref={logContainerRef}
        className="bg-black text-green-400 p-3 rounded font-mono text-sm overflow-y-auto h-64 whitespace-pre-wrap"
      >
        {isLoading && !logs && (
          <div className="flex items-center justify-center h-full">
            <Loader2 className="size-6 animate-spin mr-2" />
            Starting training process...
          </div>
        )}
        {error && <div className="text-red-400">Error: {error}</div>}
        {logs || "Waiting for logs..."}
      </div>
    </Card>
  );
};
