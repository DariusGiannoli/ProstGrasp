import { Button } from "@/components/ui/button";
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from "@/components/ui/tooltip";
import { Copy } from "lucide-react";
import { useState } from "react";

export function CopyButton({
  text,
  hint,
  className,
  variant = "ghost",
}: {
  text: string;
  hint: string;
  className?: string;
  variant?: "outline" | "ghost" | "default" | "link";
}) {
  const [isCopied, setIsCopied] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(text);
    setIsCopied(true);
    setTimeout(() => setIsCopied(false), 1500); // Hide the tooltip after 1.5 seconds
  };

  return (
    <TooltipProvider>
      <Tooltip open={isCopied}>
        <TooltipTrigger asChild>
          <Button
            onClick={handleCopy}
            title={hint}
            aria-label={hint}
            variant={variant}
            size="icon"
            className={className}
          >
            <Copy className="h-4 w-4" />
            <span className="sr-only">Copy</span>
          </Button>
        </TooltipTrigger>
        <TooltipContent>Copied!</TooltipContent>
      </Tooltip>
    </TooltipProvider>
  );
}
