import { SupabaseClient, createClient } from "@supabase/supabase-js";

// pull in your env vars
const SUPABASE_URL = import.meta.env.VITE_SUPABASE_URL as string | undefined;
const SUPABASE_KEY = import.meta.env.VITE_SUPABASE_KEY as string | undefined;

/**
 * If both URL and KEY are present, return a real client.
 * Otherwise return a dummy object matching the SupabaseClient shape,
 * where every method just returns a resolved `{ data: null, error: null }`.
 */
function getSupabaseClient(): SupabaseClient {
  if (SUPABASE_URL && SUPABASE_KEY) {
    return createClient(SUPABASE_URL, SUPABASE_KEY);
  }

  // Dummy no-op client
  const noOp = () => Promise.resolve({ data: null, error: null });
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const dummy: any = {
    // intercept queries: from().select().insert()… etc
    // eslint-disable-next-line @typescript-eslint/no-unused-vars
    from: (_: string) => ({
      select: noOp,
      insert: noOp,
      update: noOp,
      delete: noOp,
      // chainable helpers
      eq: () => dummy.from,
      neq: () => dummy.from,
      // add any other operators you use…
    }),
    // stub out auth calls
    auth: {
      signIn: noOp,
      signUp: noOp,
      signOut: noOp,
      user: () => null,
      session: () => null,
    },
    // stub out storage, functions, etc. if you use them
    storage: {
      from: () => ({
        upload: noOp,
        download: noOp,
      }),
    },
    functions: {
      invoke: noOp,
    },
  };

  return dummy as SupabaseClient;
}

const supabase = getSupabaseClient();
export default supabase;
